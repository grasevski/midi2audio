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
    pins, Adc, Peripherals,
};
use arrayvec::ArrayVec;
use avr_device::{asm::sleep, interrupt};
use core::{cell::RefCell, cmp::min, convert::TryFrom};
//use embedded_hal::serial::Read;
use panic_halt as _;
use wmidi::{FromBytesError, MidiMessage};

/// Midi bytes are buffered while DSP is happening.
type Midi = ArrayVec<u8, 4>;

/// Midi interface.
static SERIAL: interrupt::Mutex<RefCell<Option<Usart0<MHz16>>>> =
    interrupt::Mutex::new(RefCell::new(None));

/// Midi input FIFO message buffer.
static MIDI_IN: interrupt::Mutex<RefCell<Midi>> =
    interrupt::Mutex::new(RefCell::new(Midi::new_const()));

/// Midi output LIFO message buffer.
static MIDI_OUT: interrupt::Mutex<RefCell<Midi>> =
    interrupt::Mutex::new(RefCell::new(Midi::new_const()));

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
        let data = serial.as_mut().unwrap().read_byte();
        MIDI_IN.borrow(cs).borrow_mut().push(data);
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

/// Loops over audio input and maps to output.
#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    dp.ADC.adcsra.write(|w| w.adie().set_bit());
    let pins = pins!(dp);
    pins.d5.into_output();
    pins.d6.into_output();
    let tc0 = dp.TC0;
    tc0.tccr0a.write(|w| {
        w.com0a().match_clear();
        w.com0b().match_clear();
        w.wgm0().pwm_phase()
    });
    tc0.tccr0b.write(|w| w.cs0().direct());
    tc0.ocr0a.write(|w| unsafe { w.bits(128) });
    let mut adc = Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc);
    //let mut serial = default_serial!(dp, pins, 31250);
    //serial.listen(Event::RxComplete);
    //serial.listen(Event::DataRegisterEmpty);
    //let mut serial = default_serial!(dp, pins, 57600);
    //interrupt::free(|cs| SERIAL.borrow(cs).replace(Some(serial)));
    unsafe { interrupt::enable() };
    let (mut synth, mut _midi_out) = (Synth::default(), Midi::new_const());
    loop {
        while let Ok(audio_in) = adc.read_nonblocking(&a0) {
            let midi_in: Midi = interrupt::free(|cs| {
                //let mut serial = SERIAL.borrow(cs).borrow_mut();
                //if !midi_out.is_empty() {
                //    serial.as_mut().unwrap().write_byte(midi_out[0]);
                //    let mut midi = MIDI_OUT.borrow(cs).borrow_mut();
                //    for &byte in midi_out[1..].iter().rev() {
                //        midi.push(byte);
                //    }
                //}
                let mut midi = MIDI_IN.borrow(cs).borrow_mut();
                //while let Ok(byte) = serial.as_mut().unwrap().read() {
                //    midi.push(byte);
                //}
                midi.drain(..).collect()
            });
            let (audio, _midi) = synth.step(audio_in, &midi_in);
            tc0.ocr0b.write(|w| unsafe { w.bits(audio) });
            //ufmt::uwriteln!(&mut serial, "data: {}\r", audio_in);
            //midi_out = midi;
        }
        sleep();
    }
}

/// Digital signal processor.
#[derive(Default)]
struct Synth {
    controller: MidiController,
    synth: WavetableSynth,
    midi: ArrayVec<u8, 3>,
}

impl Synth {
    /// Converts midi and audio to midi and audio.
    fn step(&mut self, audio_in: u16, midi_in: &Midi) -> (u8, Midi) {
        let mut i = 0;
        while i < midi_in.len() {
            let j = min(self.midi.remaining_capacity(), midi_in.len() - i);
            self.midi.try_extend_from_slice(&midi_in[i..j]).unwrap();
            match MidiMessage::try_from(&self.midi[..]) {
                Ok(midi) => {
                    self.synth.read(&midi);
                    self.midi.clear();
                    i = j;
                }
                Err(err) => match err {
                    FromBytesError::NoBytes => {
                        break;
                    }
                    FromBytesError::NotEnoughBytes => {
                        break;
                    }
                    _ => {
                        self.midi.remove(0);
                        i = j;
                    }
                },
            }
        }
        let mut midi_out = Midi::default();
        if let Some(midi) = self.controller.step(audio_in) {
            unsafe {
                midi_out.set_len(self.midi.capacity());
            }
            midi.copy_to_slice(&mut midi_out[..]).unwrap();
        }
        (self.synth.step(), midi_out)
    }
}

/// TODO Converts midi to audio.
#[derive(Default)]
struct WavetableSynth {}

impl WavetableSynth {
    /// TODO Parses a midi event.
    fn read(&mut self, _msg: &MidiMessage) {}

    /// TODO Generates the next audio value.
    fn step(&mut self) -> u8 {
        0
    }
}

/// TODO Converts audio to midi.
#[derive(Default)]
struct MidiController {}

impl MidiController {
    /// TODO Generates the next midi event.
    fn step(&mut self, _audio_in: u16) -> Option<MidiMessage> {
        None
    }
}
