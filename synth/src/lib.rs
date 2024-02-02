//! Synthesizer unit testable logic.
#![cfg_attr(not(test), no_std)]
use arrayvec::ArrayVec;
use core::{
    cmp::max,
    convert::{TryFrom, TryInto},
};
use wmidi::{Channel, FromBytesError, MidiMessage, PitchBend, ProgramNumber, Velocity};

#[allow(unused_imports)]
use micromath::F32Ext as _;

/// Precomputed tables to get pitch and wavelength.
mod lookup;

/// Midi bytes are buffered while DSP is happening.
pub type Midi = ArrayVec<u8, MIDI_CAP>;

/// Number of bytes that can be buffered.
pub const MIDI_CAP: usize = MidiController::MIDI_MAX_MESSAGES * Synth::MIDI_MESSAGE_LEN;

/// ADC midpoint reading.
const MIDPOINT: u8 = 0x80;

/// Logarithm of the time window.
const LOG_WINDOW: u8 = 4;

/// Time window.
pub const WINDOW: u8 = 1 << LOG_WINDOW;

/// Number of audio samples per second.
pub const SAMPLE_RATE: u16 = 5120;

/// Export logf function to link to C code.
#[no_mangle]
pub extern "C" fn logf(x: f32) -> f32 {
    x.ln()
}

/// Export expf function to link to C code.
#[no_mangle]
pub extern "C" fn expf(x: f32) -> f32 {
    x.exp()
}

/// Digital signal processor.
#[derive(Default)]
pub struct Synth {
    /// Converts audio to midi.
    controller: MidiController,

    /// Converts midi to audio.
    synth: SubtractiveSynth,

    /// Midi input buffer.
    midi: ArrayVec<u8, { Self::MIDI_MESSAGE_LEN }>,
}

impl Synth {
    /// The number of bytes in a midi message.
    const MIDI_MESSAGE_LEN: usize = 3;

    /// Converts midi and audio to midi and audio.
    pub fn step(&mut self, audio_in: u8, midi_in: &[u8]) -> (u8, Midi) {
        for x in midi_in {
            self.midi.push(*x);
            match MidiMessage::try_from(&self.midi[..]) {
                Ok(midi) => {
                    self.synth.read(&midi);
                    self.midi.clear();
                }
                Err(err) => {
                    if !matches!(
                        err,
                        FromBytesError::NoBytes | FromBytesError::NotEnoughBytes
                    ) {
                        self.midi.remove(0);
                    }
                }
            }
        }
        let midi_out = Self::serialize_midi(&self.controller.step(audio_in));
        (self.synth.step(), midi_out)
    }

    /// Converts midi messages to bytes.
    fn serialize_midi(msgs: &[MidiMessage]) -> Midi {
        let (mut midi_out, mut n) = (Midi::from([0; MIDI_CAP]), 0);
        for x in msgs {
            n += x.copy_to_slice(&mut midi_out[n..]).unwrap();
        }
        midi_out.truncate(n);
        midi_out
    }
}

/// The number of different basic sound waves.
const NUM_WAVES: usize = 3;

/// A waveform as a function of period and time.
type Wave = fn(i16, i16) -> i16;

/// Defines the basic sound waves to generate.
struct Waves(ArrayVec<Wave, NUM_WAVES>);

impl Waves {
    /// Alternates between min and max amplitude.
    const fn square(t: i16, k: i16) -> i16 {
        if t << 1 <= k {
            i16::MIN
        } else {
            i16::MAX
        }
    }

    /// A waveform resembling the teeth of a plain toothed saw.
    const fn sawtooth(t: i16, k: i16) -> i16 {
        ((i16::MAX / k) * (t - (k >> 1))) << 1
    }

    /// A piecwise linear triangle shaped waveform.
    const fn triangle(t: i16, k: i16) -> i16 {
        (Self::sawtooth(t, k).abs() - (i16::MAX >> 1)) << 1
    }
}

impl Default for Waves {
    fn default() -> Self {
        ProgramNumber::default().into()
    }
}

impl From<ProgramNumber> for Waves {
    fn from(program_number: ProgramNumber) -> Self {
        let mut waves: ArrayVec<Wave, NUM_WAVES> = ArrayVec::new_const();
        let wave = max(u8::from(program_number) & 0b111, 1);
        if wave & 0b1 != 0 {
            waves.push(Self::square);
        }
        if wave & 0b10 != 0 {
            waves.push(Self::sawtooth);
        }
        if wave & 0b100 != 0 {
            waves.push(Self::triangle);
        }
        Self(waves)
    }
}

/// Converts midi to audio.
#[derive(Default)]
struct SubtractiveSynth {
    /// Generates the waveform.
    oscillator: Option<Oscillator>,

    /// The types of soundwaves to generate.
    waves: Waves,

    /// Filters the waveform.
    filter: BandPassFilter,

    /// The rate at which to taper off the note.
    decay: u8,

    /// The note to play.
    note: Option<wmidi::Note>,

    /// A microtonal adjustment to the note to play.
    pitch_bend: PitchBend,

    /// The volume of the note to play.
    velocity: Velocity,
}

impl SubtractiveSynth {
    /// Parses a midi event.
    fn read(&mut self, msg: &MidiMessage) {
        match *msg {
            MidiMessage::NoteOff(channel, note, velocity) => {
                if channel == Channel::Ch1 && Some(note) == self.note {
                    self.decay = u8::from(velocity);
                }
            }
            MidiMessage::NoteOn(channel, note, velocity) => {
                if channel == Channel::Ch1 {
                    self.note = Some(note);
                    self.decay = Default::default();
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
                if channel != Channel::Ch1 {
                    return;
                }
                self.waves = program_number.into();
                let program_number = u8::from(program_number);
                let l = !((program_number >> 3 & 0b11) << 4);
                let h = !(program_number >> 5 & 0b11);
                self.filter = BandPassFilter::new(l, h);
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
                *self = Self::default();
            }
            _ => {}
        }
    }

    /// Generates the next audio value.
    #[allow(clippy::cast_possible_wrap)]
    fn step(&mut self) -> u8 {
        const OFFSET: u8 = 15;
        let velocity = u8::from(self.velocity);
        if velocity <= self.decay {
            self.oscillator = None;
        } else {
            self.velocity = Velocity::from_u8_lossy(velocity - self.decay);
        }
        let (waves, filter, velocity) = (&self.waves.0[..], &mut self.filter, self.velocity);
        let ret = self.oscillator.as_mut().map_or(0, |oscillator| {
            let y = oscillator.step(waves);
            let v = i16::from(u8::from(velocity));
            (filter.step(y.to_be_bytes()[0] as i8) >> 7) * v
        });
        if ret >= 0 {
            u16::try_from(ret).unwrap() + (1 << OFFSET)
        } else {
            u16::try_from(ret - (-1 << OFFSET)).unwrap()
        }
        .to_be_bytes()[0]
    }

    /// Initializes the counter for iterating through the waveform.
    fn set_wavelength(&mut self) {
        self.oscillator = self.note.and_then(|note| {
            let pitch = i16::from(u8::from(note)) << Note::LOG_NUM_BENDS;
            let bend = i16::try_from(u16::from(self.pitch_bend)).unwrap();
            let bend = bend - i16::try_from(Note::PITCH_BEND_OFFSET).unwrap();
            let bend = bend >> (12 - Note::LOG_NUM_BENDS);
            let x = pitch + bend;
            if x >= 0 {
                let k = lookup::WAVELENGTHS[usize::try_from(x).unwrap()];
                Some(Oscillator::new(k))
            } else {
                None
            }
        });
    }
}

/// Generates the sound wave.
struct Oscillator {
    /// Time step within the sound wave.
    t: i16,

    /// Sound wave length.
    k: i16,
}

impl Oscillator {
    /// Initializes the oscillator with the given wavelength.
    const fn new(k: i16) -> Self {
        Self { t: 0, k }
    }

    /// Generates the next sample.
    fn step(&mut self, w: &[Wave]) -> i16 {
        self.t = (self.t + (1 << Note::WAVELENGTH_BITS)) % self.k;
        let (n, mut x) = (i16::try_from(w.len()).unwrap(), 0);
        for f in w {
            x += f(self.t, self.k) / n;
        }
        x
    }
}

/// Filters out frequencies outside a given range.
#[derive(Default)]
struct BandPassFilter {
    l: LowPassFilter,
    h: HighPassFilter,
}

impl BandPassFilter {
    /// Constructs the filter based on the given range.
    const fn new(l: u8, h: u8) -> Self {
        Self {
            l: LowPassFilter::new(l),
            h: HighPassFilter::new(h),
        }
    }

    /// Calculates the output of the filter.
    #[allow(clippy::cast_possible_wrap)]
    fn step(&mut self, x: i8) -> i16 {
        self.h.step(self.l.step(x).to_be_bytes()[0] as i8)
    }
}

/// Filters out high frequencies.
struct LowPassFilter {
    a: u8,
    y: i8,
}

impl Default for LowPassFilter {
    fn default() -> Self {
        Self::new(191)
    }
}

impl LowPassFilter {
    /// Constructs the filter based on the given cutoff.
    const fn new(a: u8) -> Self {
        Self { a, y: 0 }
    }

    /// Calculates the output of the filter.
    #[allow(clippy::cast_possible_wrap)]
    fn step(&mut self, x: i8) -> i16 {
        let a = i16::from(self.a) + 1;
        let ret = a * i16::from(x) + (256 - a) * i16::from(self.y);
        self.y = ret.to_be_bytes()[0] as i8;
        ret
    }
}

/// Filters out low frequencies.
struct HighPassFilter {
    a: u8,
    x: i8,
    y: i8,
}

impl Default for HighPassFilter {
    fn default() -> Self {
        Self::new(255)
    }
}

impl HighPassFilter {
    /// Constructs the filter based on the given cutoff.
    const fn new(a: u8) -> Self {
        Self { a, x: 0, y: 0 }
    }

    /// Calculates the output of the filter.
    #[allow(clippy::cast_possible_wrap)]
    fn step(&mut self, x: i8) -> i16 {
        let a = i16::from(self.a) + 1;
        let ret = a.saturating_mul(i16::from(self.y) + i16::from(x) - i16::from(self.x));
        self.x = x;
        self.y = ret.to_be_bytes()[0] as i8;
        ret
    }
}

/// Converts audio to midi.
#[derive(Default)]
struct MidiController {
    /// Time counter.
    t: u8,

    /// Previous pitch.
    f: Option<Note>,

    /// Calculates the amplitude over time.
    amplitude: AmplitudeTracker,

    /// Filters the sound prior to frequency tracking.
    filter: BandPassFilter,

    /// Calculates the frequency over time.
    frequency: FrequencyTracker,
}

impl MidiController {
    /// Maximum number of midi messages to send in one go.
    const MIDI_MAX_MESSAGES: usize = 3;

    /// Generates the next midi event.
    fn step(&mut self, audio_in: u8) -> ArrayVec<MidiMessage, { Self::MIDI_MAX_MESSAGES }> {
        self.t += 1;
        self.amplitude.step(audio_in);
        let audio_in = i16::from(audio_in) - i16::from(MIDPOINT);
        let audio_in = self.filter.step(i8::try_from(audio_in).unwrap());
        let audio_in = (audio_in >> 8) + i16::from(MIDPOINT);
        self.frequency.step(u8::try_from(audio_in).unwrap());
        let mut ret = ArrayVec::new_const();
        if self.t < WINDOW {
            return ret;
        }
        let a = self.amplitude.calculate();
        let f = if a == Default::default() {
            None
        } else {
            self.frequency.calculate()
        };
        if let Some(f) = f {
            let note_on = MidiMessage::NoteOn(Channel::Ch1, f.note, a);
            let pitch_bend_change = MidiMessage::PitchBendChange(Channel::Ch1, f.pitch_bend);
            if let Some(p) = self.f {
                let (p_note, f_note) = (u8::from(p.note), u8::from(f.note));
                if p_note >= f_note - 2 && p_note <= f_note + 1 {
                    let pitch_bend = i16::try_from(u16::from(f.pitch_bend)).unwrap()
                        + 0x1000 * (i16::from(p_note) - i16::from(f_note));
                    let pitch_bend =
                        PitchBend::try_from(u16::try_from(pitch_bend).unwrap()).unwrap();
                    ret.push(MidiMessage::PitchBendChange(Channel::Ch1, pitch_bend));
                    ret.push(MidiMessage::ChannelPressure(Channel::Ch1, a));
                } else {
                    ret.push(MidiMessage::NoteOff(Channel::Ch1, p.note, a));
                    ret.push(pitch_bend_change);
                    ret.push(note_on);
                }
            } else {
                ret.push(pitch_bend_change);
                ret.push(note_on);
            }
        } else if let Some(f) = self.f {
            ret.push(MidiMessage::NoteOff(Channel::Ch1, f.note, a));
        }
        self.f = f;
        self.t = 0;
        ret
    }
}

/// Simple Average Rectified Value amplitude calculator.
#[derive(Default)]
struct AmplitudeTracker(u16);

impl AmplitudeTracker {
    /// Accumulates the absolute values of the audio wave.
    fn step(&mut self, audio_in: u8) {
        let a = if audio_in >= MIDPOINT {
            audio_in - MIDPOINT
        } else {
            MIDPOINT - audio_in
        };
        self.0 += u16::from(a);
    }

    /// Takes the mean of the absolute values of the audio wave.
    fn calculate(&mut self) -> Velocity {
        const MAX_LOG_WINDOW: u8 = 9;
        static_assertions::const_assert!(
            (WINDOW as f32) * 3125.0 / (SAMPLE_RATE as f32) > (MIDI_CAP as f32)
        );
        static_assertions::const_assert!(LOG_WINDOW <= MAX_LOG_WINDOW);
        let v = if LOG_WINDOW == MAX_LOG_WINDOW {
            self.0 >> 1
        } else {
            self.0 << (i16::from(MAX_LOG_WINDOW - LOG_WINDOW) - 1)
        };
        self.0 = 0;
        Velocity::from_u8_lossy(v.to_be_bytes()[0])
    }
}

/// Pitch detector using nonlinear least squares.
struct FrequencyTracker(ArrayVec<f32, { fastf0nls::SAMPLE_SIZE as usize }>);

impl Default for FrequencyTracker {
    fn default() -> Self {
        let mut ret = ArrayVec::from([0.0; { fastf0nls::SAMPLE_SIZE as usize }]);
        ret.truncate(fastf0nls::SAMPLE_SIZE as usize - usize::from(WINDOW));
        Self(ret)
    }
}

impl FrequencyTracker {
    /// Accumulates the audio sample into an array of floats.
    fn step(&mut self, audio_in: u8) {
        self.0.push(f32::from(audio_in) / 128.0 - 1.0);
    }

    /// Estimates pitch using nonlinear least squares.
    fn calculate(&mut self) -> Option<Note> {
        let f = unsafe { fastf0nls::fastf0nls(self.0.as_ptr()) };
        const F: f32 = (SAMPLE_RATE << (Note::LOG_FREQUENCY_DIVISOR - 1)) as f32;
        let f = (F / (2.0 * core::f32::consts::PI)) * f;
        self.0.drain(..usize::from(WINDOW));
        Note::try_from_frequency(f.round() as usize)
    }
}

/// A musical pitch and accompanying bend.
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct Note {
    /// A musical pitch.
    note: wmidi::Note,

    /// Microtonal offset.
    pitch_bend: PitchBend,
}

impl Note {
    /// Base 2 logarithm of the number of subdivisions per semitone.
    pub const LOG_NUM_BENDS: u8 = 1;

    /// The number of subdivisions per octave.
    pub const OCTAVE: u8 = 12 << Self::LOG_NUM_BENDS;

    /// Tune everything with respect to A.
    pub const BASE_NOTE: wmidi::Note = wmidi::Note::A1;

    /// Frequency of A.
    pub const BASE_FREQUENCY: f64 = 55.0;

    /// Use the lower bits as the mantissa.
    pub const WAVELENGTH_BITS: u8 = 5;

    /// Base 2 logarithm of the frequency resolution.
    pub const LOG_FREQUENCY_DIVISOR: u8 = 1;

    /// A semitone pitch bend.
    const SEMITONE: u16 = 0x1000;

    /// Pitch bend midpoint.
    const PITCH_BEND_OFFSET: u16 = Self::SEMITONE << 1;

    /// Tries to convert a frequency to a note.
    fn try_from_frequency(f: usize) -> Option<Self> {
        if f >= lookup::NOTES.len() {
            return None;
        }
        let p = lookup::NOTES[f];
        if p < 0 {
            return None;
        }
        let p = u8::try_from(p).unwrap();
        let pitch_bend = (Self::PITCH_BEND_OFFSET
            + ((u16::from(p) & ((1 << Self::LOG_NUM_BENDS) - 1)) << (12 - Self::LOG_NUM_BENDS)))
            .try_into()
            .unwrap();
        let note =
            wmidi::Note::try_from(u8::from(Self::BASE_NOTE) + (p >> Self::LOG_NUM_BENDS)).unwrap();
        Some(Self { note, pitch_bend })
    }
}

#[cfg(test)]
mod tests {
    use super::{lookup, Note, Synth, LOG_WINDOW, MIDI_CAP, MIDPOINT, WINDOW};
    use core::convert::TryFrom;
    use proptest::prelude::*;
    use wmidi::{Channel, MidiMessage, PitchBend, ProgramNumber, Velocity};

    /// Check that converting wavelength to frequency to pitch to wavelength works.
    #[test]
    fn lookup() {
        let base_note = usize::from(u8::from(Note::BASE_NOTE)) << Note::LOG_NUM_BENDS;
        let max_note = base_note + 5 * usize::from(Note::OCTAVE);
        const NUMERATOR: u32 =
            (SAMPLE_RATE as u32) << (Note::LOG_FREQUENCY_DIVISOR + Note::WAVELENGTH_BITS);
        for &actual in &lookup::WAVELENGTHS[base_note..max_note] {
            let f = usize::try_from(NUMERATOR / u32::try_from(actual).unwrap()).unwrap();
            let p = usize::try_from(lookup::NOTES[f]).unwrap() + base_note;
            let expected = lookup::WAVELENGTHS[p];
            assert_eq!(actual, expected);
        }
    }

    proptest! {
        /// Check that input is handled gracefully.
        #[test]
        fn fuzz(steps in prop::collection::vec((0_u8.., prop::collection::vec(0_u8.., 0..MIDI_CAP)), 0..1000)) {
            let mut synth = Synth::default();
            for (audio_in, midi_in) in steps {
                synth.step(audio_in, &midi_in);
            }
        }

        /// Check that converting midi to audio and back works.
        #[test]
        fn roundtrip(program_number in 0_u8..0x80, pitch_bend in 0_u16..(1 << Note::LOG_NUM_BENDS), note in 36_u8..109, velocity in 1_u8..0x80, off_velocity in 1_u8..0x80) {
            const PITCH_BEND_MULTIPLE: u16 = 1 << (12 - Note::LOG_NUM_BENDS);
            let program_number = ProgramNumber::from_u8_lossy(program_number);
            let pitch_bend = PitchBend::try_from(Note::PITCH_BEND_OFFSET + PITCH_BEND_MULTIPLE * pitch_bend).unwrap();
            let note = wmidi::Note::from_u8_lossy(note);
            let velocity = Velocity::from_u8_lossy(velocity);
            let off_velocity = Velocity::from_u8_lossy(off_velocity);
            let (mut midi2audio, mut audio2midi) = (Synth::default(), Synth::default());
            let program_change = MidiMessage::ProgramChange(Channel::Ch1, program_number);
            let pitch_bend = MidiMessage::PitchBendChange(Channel::Ch1, pitch_bend);
            let note_on = MidiMessage::NoteOn(Channel::Ch1, note, velocity);
            let midi_in = [program_change, pitch_bend.clone(), note_on.clone()];
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &Synth::serialize_midi(&midi_in));
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
            prop_assert_eq!(audio_out, MIDPOINT);
            prop_assert!(midi_out.is_empty());
            for _ in 0..(WINDOW - 2) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
                prop_assert_eq!(audio_out, MIDPOINT);
                prop_assert!(midi_out.is_empty());
            }
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
            prop_assert_eq!(audio_out, MIDPOINT);
            let note_on = Synth::serialize_midi(&[pitch_bend.clone(), note_on]);
            prop_assert_eq!(&midi_out[..5], &note_on[..5]);
            let msg = MidiMessage::NoteOff(Channel::Ch1, note, off_velocity);
            let note_off = Synth::serialize_midi(&[msg]);
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &note_off);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
            prop_assert_eq!(audio_out, MIDPOINT);
            prop_assert!(midi_out.is_empty());
            for _ in 0..(WINDOW - 2) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
                prop_assert_eq!(audio_out, MIDPOINT);
                prop_assert!(midi_out.is_empty());
            }
            let channel_pressure = MidiMessage::ChannelPressure(Channel::Ch1, velocity);
            let mut channel_pressure = Synth::serialize_midi(&[pitch_bend, channel_pressure]);
            channel_pressure[2] = 0;
            channel_pressure[4] = 0;
            let velocity = u16::from(u8::from(velocity));
            let off_velocity = u16::from(u8::from(off_velocity));
            for _ in 0..(velocity/(off_velocity << LOG_WINDOW)) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
                prop_assert!(midi_out.is_empty());
                let (audio_out, mut midi_out) = audio2midi.step(audio_out, &[]);
                prop_assert_eq!(audio_out, MIDPOINT);
                midi_out[2] = 0;
                midi_out[4] = 0;
                prop_assert_eq!(&midi_out, &channel_pressure);
                for _ in 0..(WINDOW - 1) {
                    let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
                    prop_assert!(midi_out.is_empty());
                    let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
                    prop_assert_eq!(audio_out, MIDPOINT);
                    prop_assert!(midi_out.is_empty());
                }
            }
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
            prop_assert_eq!(audio_out, MIDPOINT);
            prop_assert_eq!(&midi_out[..2], &note_off[..2]);
            for _ in 0..(WINDOW - 1) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &[]);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(audio_out, &[]);
                prop_assert_eq!(audio_out, MIDPOINT);
                prop_assert!(midi_out.is_empty());
            }
        }
    }
}
