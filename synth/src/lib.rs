//! Synthesizer unit testable logic.
#![cfg_attr(not(feature = "std"), no_std)]
use arrayvec::ArrayVec;
use core::{
    cmp::{max, min},
    convert::{TryFrom, TryInto},
};
use wmidi::{Channel, FromBytesError, MidiMessage, PitchBend, ProgramNumber, Velocity};

mod lookup;

/// Midi bytes are buffered while DSP is happening.
pub type Midi = ArrayVec<u8, 4>;

/// ADC midpoint reading.
const MIDPOINT: u16 = 512;

/// Generates the frequency to pitch mapping.
#[cfg(feature = "std")]
pub fn lookup_notes() -> [i16; Note::NUM_NOTES] {
    let ret: ArrayVec<_, { Note::NUM_NOTES }> = (0..Note::NUM_NOTES).map(Note::lookup).collect();
    ret.into_inner().unwrap()
}

/// Generates the pitch to wavelength mapping.
#[cfg(feature = "std")]
pub fn lookup_wavelengths() -> [i16; Note::NUM_WAVELENGTHS] {
    let ret: ArrayVec<_, { Note::NUM_WAVELENGTHS }> = (0..Note::NUM_WAVELENGTHS)
        .map(Note::lookup_wavelength)
        .collect();
    ret.into_inner().unwrap()
}

/// Digital signal processor.
#[derive(Default)]
pub struct Synth<'a> {
    /// Converts audio to midi.
    controller: MidiController<'a>,

    /// Converts midi to audio.
    synth: SubtractiveSynth,

    /// Midi input buffer.
    midi: ArrayVec<u8, 3>,
}

impl Synth<'_> {
    /// Converts midi and audio to midi and audio.
    pub fn step(&mut self, audio_in: u16, midi_in: &Midi) -> (u8, Midi) {
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
    /// Time step within the sound wave.
    t: i16,

    /// Sound wave length.
    k: i16,

    /// Previous amplitude, for doing low pass filter.
    p: i16,

    /// The note to play.
    note: Option<wmidi::Note>,

    /// A microtonal adjustment to the note to play.
    pitch_bend: PitchBend,

    /// The selected musical instrument.
    program_number: ProgramNumber,

    /// The volume of the note to play.
    velocity: Velocity,
}

impl SubtractiveSynth {
    /// Parses a midi event.
    fn read(&mut self, msg: &MidiMessage) {
        match *msg {
            MidiMessage::NoteOff(channel, note, _) => {
                if channel == Channel::Ch1 && Some(note) == self.note {
                    self.note = None;
                    self.set_wavelength();
                }
            }
            MidiMessage::NoteOn(channel, note, velocity) => {
                if channel == Channel::Ch1 {
                    self.note = Some(note);
                    self.set_wavelength();
                    self.velocity = velocity;
                }
            }
            MidiMessage::PolyphonicKeyPressure(channel, note, velocity) => {
                if channel == Channel::Ch1 && Some(note) == self.note {
                    self.velocity = velocity;
                }
            }
            MidiMessage::ProgramChange(channel, program_number) => {
                if channel == Channel::Ch1 {
                    self.program_number = program_number;
                }
            }
            MidiMessage::ChannelPressure(channel, velocity) => {
                if channel == Channel::Ch1 {
                    self.velocity = velocity;
                }
            }
            MidiMessage::PitchBendChange(channel, pitch_bend) => {
                if channel == Channel::Ch1 {
                    self.pitch_bend = pitch_bend;
                    self.set_wavelength();
                }
            }
            MidiMessage::Reset => {
                *self = Default::default();
            }
            _ => {}
        }
    }

    /// Generates the next audio value.
    fn step(&mut self) -> u8 {
        self.p = if self.k == 0 {
            0
        } else {
            let program_number = u8::from(self.program_number);
            let wave = max(program_number & 0b111, 1);
            const MAX_ALPHA: u8 = 16;
            let a = i16::from(MAX_ALPHA - ((program_number >> 3) & 0b1111));
            let v = ((i16::MAX - 1) >> 11) * a * i16::from(u8::from(self.velocity))
                / i16::try_from(wave.count_ones()).unwrap();
            let mut r = 0_i16;
            if wave & 0b1 != 0 {
                r += Self::square(self.t, self.k, v);
            }
            if wave & 0b10 != 0 {
                r += Self::sawtooth(self.t, self.k, v);
            }
            if wave & 0b100 != 0 {
                r += Self::triangle(self.t, self.k, v);
            }
            self.t = (self.t + 1) % self.k;
            r + (i16::from(MAX_ALPHA) - a) * (self.p >> 4)
        };
        u8::try_from((self.p >> 9) + 128).unwrap()
    }

    /// Initializes the counter for iterating through the waveform.
    fn set_wavelength(&mut self) {
        self.t = 0;
        self.k = if let Some(note) = self.note {
            let pitch = i16::from(u8::from(note)) << 3;
            let bend = i16::try_from(u16::from(self.pitch_bend) >> 11).unwrap();
            lookup::WAVELENGTHS[usize::try_from(pitch + bend).unwrap()]
        } else {
            0
        }
    }

    /// Alternates between min and max amplitude.
    fn square(t: i16, k: i16, v: i16) -> i16 {
        v * if (t << 1) < k { -1 } else { 1 }
    }

    /// A waveform resembling the teeth of a plain toothed saw.
    fn sawtooth(t: i16, k: i16, v: i16) -> i16 {
        (v / k) * ((t << 1) - k)
    }

    /// A piecwise linear triangle shaped waveform.
    fn triangle(t: i16, k: i16, v: i16) -> i16 {
        let x = if (t << 1) < k {
            (t << 2) - k
        } else {
            k + (k >> 1) - (t << 2)
        };
        (v / k) * x
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
            Note::try_from_frequency((f >> 1).into())
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
    /// Number of audio samples per second.
    const NUM_SAMPLES: u16 = 8192;

    /// Pitch bend neutral value.
    const PITCH_OFFSET: u16 = 8192;

    /// The number of distinct pitches that can be recognized.
    const NUM_NOTES: usize = 8192;

    /// The number of distinct wavelengths represented by midi notes.
    const NUM_WAVELENGTHS: usize = 2048;

    /// The number of subdivisions per semitone.
    const NUM_BENDS: i16 = 8;

    /// The number of subdivisions per octave.
    const DENOMINATOR: f64 = 12.0 * (Self::NUM_BENDS as f64);

    /// Tune everything with respect to A.
    const BASE_NOTE: wmidi::Note = wmidi::Note::A0;

    /// Frequency of A.
    const BASE_FREQUENCY: f64 = 55.0;

    /// Tries to convert a frequency to a note.
    fn try_from_frequency(f: usize) -> Option<Self> {
        if f >= lookup::NOTES.len() {
            return None;
        }
        let p = lookup::NOTES[f];
        if p < 0 {
            return None;
        }
        let p = u16::try_from(p).unwrap();
        let pitch_bend = (Self::PITCH_OFFSET + ((p & 0b1111) << 9))
            .try_into()
            .unwrap();
        let note = wmidi::Note::try_from(u8::from(Self::BASE_NOTE) + u8::try_from(p >> 4).unwrap())
            .unwrap();
        Some(Note { note, pitch_bend })
    }

    /// Maps a given frequency to its pitch.
    #[cfg(feature = "std")]
    fn lookup(frequency: usize) -> i16 {
        (Self::DENOMINATOR
            * (f64::try_from(i16::try_from(frequency).unwrap()).unwrap() / Self::BASE_FREQUENCY)
                .log2())
        .round() as i16
    }

    /// Maps a given pitch to its wavelength.
    #[cfg(feature = "std")]
    fn lookup_wavelength(pitch: usize) -> i16 {
        let p =
            i16::try_from(pitch).unwrap() - Self::NUM_BENDS * i16::from(u8::from(Self::BASE_NOTE));
        (f64::try_from(Self::NUM_SAMPLES).unwrap()
            / (Self::BASE_FREQUENCY * (f64::try_from(p).unwrap() / Self::DENOMINATOR).exp2()))
        .round() as i16
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
