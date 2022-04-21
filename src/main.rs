//! Arduino synthesizer.
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(const_fn_floating_point_arithmetic)]
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
use core::{
    cell::RefCell,
    cmp::min,
    convert::{TryFrom, TryInto},
};
//use embedded_hal::serial::Read;
use panic_halt as _;
use wmidi::{Channel, FromBytesError, MidiMessage, PitchBend, ProgramNumber, Velocity};

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

/// ADC midpoint reading.
const MIDPOINT: u16 = 512;

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
struct Synth<'a> {
    /// Converts audio to midi.
    controller: MidiController<'a>,

    /// Converts midi to audio.
    synth: SubtractiveSynth,

    /// Midi input buffer.
    midi: ArrayVec<u8, 3>,
}

impl Synth<'_> {
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

/// Converts midi to audio.
#[derive(Default)]
struct SubtractiveSynth {
    t: u16,
    k: u16,
    program_number: ProgramNumber,
    velocity: Velocity,
}

impl SubtractiveSynth {
    /// Parses a midi event.
    fn read(&mut self, msg: &MidiMessage) {
        match *msg {
            MidiMessage::NoteOff(channel, note, _) => {
                if channel == Channel::Ch1 && Some(note) == self.note {
                    self.note = None;
                }
            },
            MidiMessage::NoteOn(channel, note, velocity) => {
                if channel == Channel::Ch1 {
                    self.note = Some(note);
                    self.velocity = velocity;
                }
            },
            MidiMessage::PolyphonicKeyPressure(channel, note, velocity) => {
                if channel == Channel::Ch1 && Some(note) == self.note {
                    self.velocity = velocity;
                }
            },
            MidiMessage::ProgramChange(channel, program_number) => {
                if channel == Channel::Ch1 {
                    self.program_number = program_number;
                }
            },
            MidiMessage::ChannelPressure(channel, velocity) => {
                if channel == Channel::Ch1 {
                    self.velocity = velocity;
                }
            },
            MidiMessage::PitchBendChange(channel, pitch_bend) => {
                if channel == Channel::Ch1 {
                    self.pitch_bend = pitch_bend;
                }
            },
            MidiMessage::Reset => {
                *self = Default::default();
            },
            _ => {},
        }
    }

    /// TODO Generates the next audio value.
    fn step(&mut self) -> u8 {
        if let Some(note) = self.note {
            let pitch = i16::from(u8::from(note)) << 3;
            let bend = i16::try_from(u16::from(self.pitch_bend) >> 11).unwrap();
            let f = Note::PITCHES[usize::try_from(pitch + bend).unwrap()];
            let program_number = u8::from(self.program_number);
            let mut c = 0;
            if program_number & 0b1 != 0 {
                c += 1;
            }
            if program_number & 0b10 != 0 {
                c += 1;
            }
            if program_number & 0b100 != 0 || c == 0 {
                c += 1;
            }
        }
        0
    }
}

/// Converts audio to midi.
#[derive(Default)]
struct MidiController<'a> {
    /// Time counter.
    t: u8,

    /// Previous pitch.
    f: Option<Note>,

    /// Calculates the amplitude over time.
    amplitude: AmplitudeTracker,

    /// Calculates the frequency over time.
    frequency: FrequencyTracker,

    /// Midi output buffer.
    msgs: ArrayVec<MidiMessage<'a>, 3>,
}

impl MidiController<'_> {
    /// Generates the next midi event.
    fn step(&mut self, audio_in: u16) -> Option<MidiMessage> {
        self.t += 1;
        self.amplitude.step(audio_in);
        self.frequency.step(audio_in);
        if self.t < 64 {
            return self.msgs.pop();
        }
        let a = self.amplitude.calculate();
        let f = self.frequency.calculate();
        self.amplitude = Default::default();
        self.frequency = Default::default();
        if let Some(f) = f {
            let note_on = MidiMessage::NoteOn(Channel::Ch1, f.note, a);
            let pitch_bend_change = MidiMessage::PitchBendChange(Channel::Ch1, f.pitch_bend);
            if let Some(p) = self.f {
                if p.note == f.note {
                    self.msgs
                        .push(MidiMessage::ChannelPressure(Channel::Ch1, a));
                    self.msgs.push(pitch_bend_change);
                } else {
                    self.msgs.push(note_on);
                    self.msgs.push(pitch_bend_change);
                    self.msgs.push(MidiMessage::NoteOff(
                        Channel::Ch1,
                        p.note,
                        Default::default(),
                    ));
                }
            } else {
                self.msgs.push(note_on);
                self.msgs.push(pitch_bend_change);
            }
        } else if let Some(f) = self.f {
            self.msgs.push(MidiMessage::NoteOff(
                Channel::Ch1,
                f.note,
                Default::default(),
            ));
        }
        self.f = f;
        self.t = 0;
        self.msgs.pop()
    }
}

/// Simple Average Rectified Value amplitude calculator.
#[derive(Default)]
struct AmplitudeTracker(u16);

impl AmplitudeTracker {
    /// Accumulates the absolute values of the audio wave.
    fn step(&mut self, audio_in: u16) {
        self.0 += if audio_in >= MIDPOINT {
            audio_in - MIDPOINT
        } else {
            MIDPOINT - audio_in
        };
    }

    /// Takes the mean of the absolute values of the audio wave.
    fn calculate(&self) -> Velocity {
        Velocity::from_u8_lossy(self.0.to_be_bytes()[0])
    }
}

/// Simple zero crossing rate monophonic pitch detector.
#[derive(Default)]
struct FrequencyTracker {
    /// Whether the previous point was positive.
    p: Option<bool>,

    /// The position of the first crossing.
    i: u8,

    /// The position of the last crossing.
    f: u8,

    /// Number of crossings.
    k: u8,

    /// The current position.
    n: u8,
}

impl FrequencyTracker {
    /// Counts the number of zero crossings and where they start and end.
    fn step(&mut self, audio_in: u16) {
        let p = audio_in >= MIDPOINT;
        if let Some(x) = self.p {
            if x != p {
                if self.k == 0 {
                    self.i = self.n;
                }
                self.k += 1;
                self.f = self.n;
            }
        }
        self.p = Some(p);
        self.n += 1;
    }

    /// Estimates pitch based on number of zero crossings and where they start and end.
    fn calculate(&self) -> Option<Note> {
        if self.i == self.f {
            Default::default()
        } else {
            let f = u16::from(self.k - 1) * (32768 / u16::from(self.f - self.i));
            Note::try_from_frequency((f >> 2).into())
        }
    }
}

/// A musical pitch and accompanying bend.
#[derive(Clone, Copy, Eq, PartialEq)]
struct Note {
    /// A musical pitch.
    note: wmidi::Note,

    /// Microtonal offset.
    pitch_bend: PitchBend,
}

impl Note {
    /// The number of distinct pitches that can be recognized.
    const NUM_NOTES: usize = 4096;

    /// The number of subdivisions per semitone.
    const NUM_BENDS: i16 = 8;

    /// The number of subdivisions per octave.
    const DENOMINATOR: f64 = 12.0 * (Self::NUM_BENDS as f64);

    /// Tune everything with respect to A.
    const BASE_NOTE: wmidi::Note = wmidi::Note::A0;

    /// Frequency of A.
    const BASE_FREQUENCY: f64 = 55.0;

    /// A precomputed pitch mapping table.
    const NOTES: [i16; Self::NUM_NOTES] = Self::lookup();

    /// A precomputed frequency mapping table.
    const PITCHES: [u16; Self::NUM_NOTES] = Self::lookup_pitch();

    /// Tries to convert a frequency to a note.
    fn try_from_frequency(f: usize) -> Option<Self> {
        if f >= Self::NOTES.len() {
            return None;
        }
        let p = Self::NOTES[f];
        if p < 0 {
            return None;
        }
        let p = u16::try_from(p).unwrap();
        let pitch_bend = (8192 + ((p & 0b111) << 9)).try_into().unwrap();
        let note = wmidi::Note::try_from(u8::from(Self::BASE_NOTE) + u8::try_from(p >> 3).unwrap()).unwrap();
        Some(Note { note, pitch_bend })
    }

    /// Generates the frequency to pitch mapping.
    const fn lookup() -> [i16; Self::NUM_NOTES] {
        let mut ret = [0; Self::NUM_NOTES];
        let mut i = 0;
        while i < ret.len() {
            ret[i] = (Self::DENOMINATOR * ((i as f64) / Self::BASE_FREQUENCY).log2()) as i16;
            i += 1;
        }
        ret
    }

    /// Generates the pitch to frequency mapping.
    const fn lookup_pitch() -> [u16; Self::NUM_NOTES] {
        let mut ret = [0; Self::NUM_NOTES];
        let mut i = 0;
        while i < ret.len() {
            let p = (i as i16) - Self::NUM_BENDS * (Self::BASE_NOTE as i16);
            ret[i] = (Self::BASE_FREQUENCY * ((p as f64) / Self::DENOMINATOR).exp2()) as u16;
            i += 1;
        }
        ret
    }
}
