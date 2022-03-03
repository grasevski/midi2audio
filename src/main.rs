//! Arduino synthesizer.
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
use arduino_hal::{
    default_serial, entry,
    hal::{
        clock::MHz16,
        usart::{Event, Usart0},
    },
    pins, Adc, Peripherals, Spi,
};
use arrayvec::ArrayVec;
use avr_device::{asm, interrupt};
use core::cell::RefCell;
use embedded_hal::spi::FullDuplex;
use panic_halt as _;

/// Max number of bytes that can be pending for midi.
const BUFSIZ: usize = 4;

/// Interface with the audio out DAC.
static SPI: interrupt::Mutex<RefCell<Option<Spi>>> = interrupt::Mutex::new(RefCell::new(None));

/// Midi interface.
static SERIAL: interrupt::Mutex<RefCell<Option<Usart0<MHz16>>>> =
    interrupt::Mutex::new(RefCell::new(None));

/// Midi input message buffer.
static MIDI_IN: interrupt::Mutex<RefCell<ArrayVec<u8, BUFSIZ>>> =
    interrupt::Mutex::new(RefCell::new(ArrayVec::new_const()));

/// Midi output message buffer.
static MIDI_OUT: interrupt::Mutex<RefCell<ArrayVec<u8, BUFSIZ>>> =
    interrupt::Mutex::new(RefCell::new(ArrayVec::new_const()));

/// Audio output least significant byte.
static AUDIO_OUT: interrupt::Mutex<RefCell<Option<u8>>> = interrupt::Mutex::new(RefCell::new(None));

/// Wakes up the main loop when the audio input has been read.
#[interrupt(atmega328p)]
#[allow(non_snake_case)]
fn ADC() {}

/// Grabs the next midi input byte.
#[interrupt(atmega328p)]
#[allow(non_snake_case)]
fn USART_RX() {
    interrupt::free(|cs| {
        let mut serial = SERIAL.borrow(cs).borrow_mut();
        let mut midi_in = MIDI_IN.borrow(cs).borrow_mut();
        midi_in.push(serial.as_mut().unwrap().read_byte());
    });
}

/// Sends the next midi output byte.
#[interrupt(atmega328p)]
#[allow(non_snake_case)]
fn USART_UDRE() {
    interrupt::free(|cs| {
        let mut midi_out = MIDI_OUT.borrow(cs).borrow_mut();
        if let Some(data) = midi_out.pop() {
            let mut serial = SERIAL.borrow(cs).borrow_mut();
            serial.as_mut().unwrap().write_byte(data);
        }
    });
}

/// Writes the least significant byte of audio output.
#[interrupt(atmega328p)]
#[allow(non_snake_case)]
fn SPI_STC() {
    interrupt::free(|cs| {
        let mut audio_out = AUDIO_OUT.borrow(cs).borrow_mut();
        if let Some(data) = audio_out.take() {
            let mut spi = SPI.borrow(cs).borrow_mut();
            let _ = spi.as_mut().unwrap().send(data);
        }
    });
}

/// Loops over audio input and maps to output.
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);
    dp.ADC.adcsra.write(|w| w.adie().set_bit());
    dp.SPI.spcr.write(|w| w.spe().set_bit());
    let mut adc = Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc);
    let (spi, _) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        Default::default(),
    );
    let mut serial = default_serial!(dp, pins, 31250);
    serial.listen(Event::RxComplete);
    serial.listen(Event::DataRegisterEmpty);
    interrupt::free(|cs| {
        SPI.borrow(cs).replace(Some(spi));
        SERIAL.borrow(cs).replace(Some(serial));
    });
    unsafe { interrupt::enable() };
    let mut synth = Synth::new();
    loop {
        while let Ok(audio_in) = adc.read_nonblocking(&a0) {
            let midi_in: ArrayVec<_, BUFSIZ> =
                interrupt::free(|cs| MIDI_IN.borrow(cs).borrow_mut().drain(..).collect());
            let (audio_out, midi_out) = synth.step(audio_in, midi_in.into_iter());
            interrupt::free(|cs| {
                let mut spi = SPI.borrow(cs).borrow_mut();
                let _ = spi.as_mut().unwrap().send(audio_out.0);
                *AUDIO_OUT.borrow(cs).borrow_mut() = Some(audio_out.1);
                if midi_out.is_empty() {
                    return;
                }
                let mut serial = SERIAL.borrow(cs).borrow_mut();
                serial.as_mut().unwrap().write_byte(midi_out[0]);
                let mut midi = MIDI_OUT.borrow(cs).borrow_mut();
                for &byte in midi_out[1..].iter().rev() {
                    midi.push(byte);
                }
            });
        }
        asm::sleep();
    }
}

/// Digital signal processor.
struct Synth {}

impl Synth {
    /// Initializes the state machine.
    const fn new() -> Self {
        Self {}
    }

    /// Currently just maps the audio in to out.
    fn step(
        &mut self,
        audio_in: u16,
        _midi_in: impl Iterator<Item = u8>,
    ) -> ((u8, u8), ArrayVec<u8, BUFSIZ>) {
        let [a, b] = (audio_in << 6).to_be_bytes();
        ((a, b), ArrayVec::new_const())
    }
}
