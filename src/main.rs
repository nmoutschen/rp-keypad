#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::prelude::*;
use embedded_time::{fixed_point::*, rate::*};
use hal::pac::interrupt;
use heapless::Vec;
use panic_halt as _;
use rp_keypad::hal::{self, pac, prelude::*};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use usb_device::{class_prelude::*, prelude::*};
use usbd_midi::data::byte::u7::U7;
use usbd_midi::data::midi::channel::Channel;
use usbd_midi::data::midi::message::Message;
use usbd_midi::data::midi::notes::Note;
use usbd_midi::data::usb_midi::cable_number::CableNumber;
use usbd_midi::data::usb_midi::usb_midi_event_packet::UsbMidiEventPacket;
use usbd_midi::{data::usb::constants::USB_CLASS_NONE, midi_device::MidiClass};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static FERRIS1: &[u8] = include_bytes!("../ferris1.raw");
static FERRIS2: &[u8] = include_bytes!("../ferris2.raw");

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut MIDI: Option<MidiClass<hal::usb::UsbBus>> = None;

#[derive(Clone, Copy, PartialEq)]
enum KeySwitch {
    Press(u8),
    Release(u8),
    None,
}

#[entry]
fn main() -> ! {
    // Device initialization
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let sio = hal::sio::Sio::new(pac.SIO);

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_keypad::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = rp_keypad::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup I2C bus
    let pin_sda = pins.sda.into_mode::<hal::gpio::FunctionI2C>();
    let pin_scl = pins.scl.into_mode::<hal::gpio::FunctionI2C>();
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        pin_sda,
        pin_scl,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let i2c_bus = shared_bus::BusManagerSimple::new(i2c);

    // Setup display
    let interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Load ferris
    let ferris1: ImageRawLE<BinaryColor> = ImageRawLE::new(FERRIS1, 128);
    let ferris1_img = Image::new(&ferris1, Point::zero());

    let ferris2: ImageRawLE<BinaryColor> = ImageRawLE::new(FERRIS2, 128);
    let ferris2_img = Image::new(&ferris2, Point::zero());

    let ferris_imgs = [ferris1_img, ferris2_img];

    // Setup leds
    let _leds_sclk = pins.leds_sclk.into_mode::<hal::gpio::FunctionSpi>();
    let _leds_mosi = pins.leds_mosi.into_mode::<hal::gpio::FunctionSpi>();
    let _leds_cs = pins.leds_cs.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    let mut leds_spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        (4 * 1024 * 1024).Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Setup USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }
    let usb_bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
    let midi = MidiClass::new(usb_bus_ref);
    let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .product("MIDI Test")
        .device_class(USB_CLASS_NONE)
        .build();

    unsafe {
        USB_DEVICE = Some(usb_dev);
        MIDI = Some(midi);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // Draw ferris
    let mut ferris_cur = 0;
    ferris_imgs[ferris_cur].draw(&mut display).unwrap();
    display.flush().unwrap();

    // Setup data
    // 16 bits for 16 keys
    let mut prev_keys = 0u16;
    let mut key_data = [0u8; 2];
    // 4 bytes before/after + 16 * 4 bytes of data
    let mut leds_data = [0u8; 72];

    // 32 is double what we can capture in a single pass
    let mut messages = Vec::<Message, 32>::new();

    loop {
        // Read keyboard state
        i2c_bus.acquire_i2c().write(0x20, &[0]).unwrap();
        i2c_bus.acquire_i2c().read(0x20, &mut key_data).unwrap();
        let keys = u16::from_le_bytes(key_data) ^ 0xffff;

        // Light up leds for pressed keys
        set_leds(&mut leds_data, keys, 0b00111, 0xff, 0xff, 0xff);
        leds_spi.write(&leds_data).unwrap();

        if messages.len() < 16 {
            let new_messages: Vec<Message, 16> = get_switches(prev_keys, keys)
                .iter()
                .filter_map(|s| (*s).into())
                .collect();
            messages.extend(new_messages);
            prev_keys = keys;
        }
        // Send midi data
        if let Some(message) = messages.pop() {
            // Invert ferris
            match message {
                Message::NoteOn { .. } => {
                    ferris_cur ^= 1;
                    display.clear();
                    ferris_imgs[ferris_cur].draw(&mut display).unwrap();
                    display.flush().unwrap();
                }
                _ => {}
            }

            // Send MIDI message
            let midi = unsafe { MIDI.as_mut().unwrap() };
            midi.send_message(UsbMidiEventPacket {
                cable_number: CableNumber::Cable0,
                message,
            })
            .unwrap();
        }

        // Wait
        delay.delay_ms(25);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let midi = MIDI.as_mut().unwrap();
    usb_dev.poll(&mut [midi]);
}

fn set_leds(data: &mut [u8; 72], leds: u16, brightness: u8, r: u8, g: u8, b: u8) {
    const OFFSET: usize = 4;
    for i in 0..16 {
        // Brightness
        data[i * 4 + 4] = 0b11100000 | brightness;
        // Color
        if leds & (1 << i) != 0 {
            data[i * 4 + OFFSET + 1] = b;
            data[i * 4 + OFFSET + 2] = g;
            data[i * 4 + OFFSET + 3] = r;
        } else {
            data[i * 4 + OFFSET + 1] = 0;
            data[i * 4 + OFFSET + 2] = 0;
            data[i * 4 + OFFSET + 3] = 0;
        }
    }
}

fn get_switches(prev: u16, curr: u16) -> [KeySwitch; 16] {
    let mut switches = [KeySwitch::None; 16];
    for i in 0..16 {
        if curr & (1 << i) != 0 {
            if prev & (1 << i) == 0 {
                switches[i] = KeySwitch::Press(i as u8);
            } else {
                switches[i] = KeySwitch::None;
            }
        } else {
            if prev & (1 << i) != 0 {
                switches[i] = KeySwitch::Release(i as u8);
            } else {
                switches[i] = KeySwitch::None;
            }
        }
    }
    switches
}

impl Into<Option<Message>> for KeySwitch {
    fn into(self) -> Option<Message> {
        match self {
            KeySwitch::Press(pos) => {
                Some(Message::NoteOn(Channel::Channel1, into_note(pos), U7::MAX))
            }
            KeySwitch::Release(pos) => {
                Some(Message::NoteOff(Channel::Channel1, into_note(pos), U7::MAX))
            }
            KeySwitch::None => None,
        }
    }
}

fn into_note(pos: u8) -> Note {
    match pos {
        0 => Note::E3,
        1 => Note::F3,
        2 => Note::Fs3,
        3 => Note::G3,
        4 => Note::Gs3,
        5 => Note::A3,
        6 => Note::As3,
        7 => Note::B3,
        8 => Note::C4,
        9 => Note::Cs4,
        10 => Note::D4,
        11 => Note::Ds4,
        12 => Note::E4,
        13 => Note::F4,
        14 => Note::Fs4,
        15 => Note::G4,
        _ => panic!("Invalid note position"),
    }
}
