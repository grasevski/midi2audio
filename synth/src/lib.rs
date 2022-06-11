//! Synthesizer unit testable logic.
#![cfg_attr(not(test), no_std)]
use arrayvec::ArrayVec;
use core::{
    cmp::{max, min},
    convert::{TryFrom, TryInto},
};
use wmidi::{Channel, FromBytesError, MidiMessage, PitchBend, ProgramNumber, Velocity};

/// Precomputed tables to get pitch and wavelength.
mod lookup;

/// Midi bytes are buffered while DSP is happening.
pub type Midi = ArrayVec<u8, MIDI_CAP>;

/// Number of bytes that can be buffered.
const MIDI_CAP: usize = 4;

/// ADC midpoint reading.
const MIDPOINT: u16 = 0x200;

/// The number of bytes in a midi message.
const MIDI_MESSAGE_LEN: usize = 3;

/// Logarithm of the maximum time window.
const MAX_LOG_WINDOW: u8 = 7;

/// Logarithm of the time window.
const LOG_WINDOW: u8 = 7;

static_assertions::const_assert!(LOG_WINDOW > 1);

static_assertions::const_assert!(LOG_WINDOW <= MAX_LOG_WINDOW);

/// Digital signal processor.
#[derive(Default)]
pub struct Synth<'a> {
    /// Converts audio to midi.
    controller: MidiController<'a>,

    /// Converts midi to audio.
    synth: SubtractiveSynth,

    /// Midi input buffer.
    midi: ArrayVec<u8, MIDI_MESSAGE_LEN>,
}

impl Synth<'_> {
    /// Converts midi and audio to midi and audio.
    pub fn step(&mut self, audio_in: u16, midi_in: &Midi) -> (u8, Midi) {
        let mut i = 0;
        while i < midi_in.len() {
            let j = i + min(self.midi.remaining_capacity(), midi_in.len() - i);
            self.midi.try_extend_from_slice(&midi_in[i..j]).unwrap();
            match MidiMessage::try_from(&self.midi[..]) {
                Ok(midi) => {
                    self.synth.read(&midi);
                    self.midi.clear();
                    i = j;
                }
                Err(err) => match err {
                    FromBytesError::NoBytes | FromBytesError::NotEnoughBytes => {
                        break;
                    }
                    _ => {
                        self.midi.remove(0);
                        i = j;
                    }
                },
            }
        }
        let midi_out = self
            .controller
            .step(audio_in)
            .as_ref()
            .map_or(Midi::new_const(), Self::serialize_midi);
        (self.synth.step(), midi_out)
    }

    /// Converts midi message to bytes.
    fn serialize_midi(midi: &MidiMessage) -> Midi {
        let mut midi_out = Midi::try_from(&[0; MIDI_MESSAGE_LEN][..]).unwrap();
        midi.copy_to_slice(&mut midi_out[..]).unwrap();
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
            i16::MAX - 1
        }
    }

    /// A waveform resembling the teeth of a plain toothed saw.
    const fn sawtooth(t: i16, k: i16) -> i16 {
        ((i16::MAX - 1) / k) * ((t << 1) - k)
    }

    /// A piecwise linear triangle shaped waveform.
    const fn triangle(t: i16, k: i16) -> i16 {
        let x = if t <= k >> 1 {
            (t << 2) - k
        } else {
            k + (k << 1) - (t << 2)
        };
        ((i16::MAX - 1) / k) * x
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
                let l = program_number >> 3 & 0b11;
                let mut h = program_number >> 5 & 0b11;
                let l = u8::try_from(0x100 - (0x100 >> l)).unwrap();
                if h > 0 {
                    h = 1 << (h - 1);
                }
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
        self.t = (self.t + 1) % self.k;
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
        Self::new(224)
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
        let ret = (i16::from(!self.a) + 1) * i16::from(x) + i16::from(self.a) * i16::from(self.y);
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
        Self::new(4)
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
        let a = i16::from(!self.a & 0x7f) + 1;
        let ret = a * (i16::from(self.y) + i16::from(x) - i16::from(self.x));
        self.x = x;
        self.y = ret.to_be_bytes()[0] as i8;
        ret
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

    /// Filters the sound prior to frequency tracking.
    filter: BandPassFilter,

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
        let audio_in = i16::try_from(audio_in).unwrap() - i16::try_from(MIDPOINT).unwrap();
        let audio_in = self.filter.step(i8::try_from(audio_in >> 2).unwrap());
        let audio_in = (audio_in >> 6) + i16::try_from(MIDPOINT).unwrap();
        self.frequency.step(u16::try_from(audio_in).unwrap());
        if self.t < 1 << LOG_WINDOW {
            return self.msgs.pop();
        }
        let a = self.amplitude.calculate();
        let f = self.frequency.calculate();
        self.amplitude = AmplitudeTracker::default();
        self.frequency = FrequencyTracker::default();
        if let Some(f) = f {
            let note_on = MidiMessage::NoteOn(Channel::Ch1, f.note, a);
            let pitch_bend_change = MidiMessage::PitchBendChange(Channel::Ch1, f.pitch_bend);
            if let Some(p) = self.f {
                let (p_note, f_note) = (u8::from(p.note), u8::from(f.note));
                if p_note >= f_note - 2 && p_note <= f_note + 1 {
                    self.msgs
                        .push(MidiMessage::ChannelPressure(Channel::Ch1, a));
                    let pitch_bend = i16::try_from(u16::from(f.pitch_bend)).unwrap()
                        + 0x1000 * (i16::from(p_note) - i16::from(f_note));
                    let pitch_bend =
                        PitchBend::try_from(u16::try_from(pitch_bend).unwrap()).unwrap();
                    self.msgs
                        .push(MidiMessage::PitchBendChange(Channel::Ch1, pitch_bend));
                } else {
                    self.msgs.push(note_on);
                    self.msgs.push(pitch_bend_change);
                    self.msgs
                        .push(MidiMessage::NoteOff(Channel::Ch1, p.note, a));
                }
            } else {
                self.msgs.push(note_on);
                self.msgs.push(pitch_bend_change);
            }
        } else if let Some(f) = self.f {
            self.msgs
                .push(MidiMessage::NoteOff(Channel::Ch1, f.note, a));
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
        let v = if LOG_WINDOW == MAX_LOG_WINDOW {
            self.0 >> 1
        } else {
            self.0 << (i16::from(MAX_LOG_WINDOW - LOG_WINDOW) - 1)
        };
        Velocity::from_u8_lossy(v.to_be_bytes()[0])
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
            None
        } else {
            let f = u16::from(self.k - 1) * (Note::FREQUENCY_MULTIPLE / u16::from(self.f - self.i));
            Note::try_from_frequency((f >> 1).into())
        }
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
    pub const LOG_NUM_BENDS: u8 = 0;

    /// The number of subdivisions per octave.
    pub const OCTAVE: u8 = 12 << Self::LOG_NUM_BENDS;

    /// Tune everything with respect to A.
    pub const BASE_NOTE: wmidi::Note = wmidi::Note::A1;

    /// Frequency of A.
    pub const BASE_FREQUENCY: f64 = 55.0;

    /// Number of audio samples per second.
    pub const SAMPLE_RATE: u16 = 9615;

    /// Base 2 logarithm of the frequency resolution.
    pub const LOG_FREQUENCY_DIVISOR: u8 = 1;

    /// Multiple for the inverse wavelength lookup.
    const FREQUENCY_MULTIPLE: u16 = Self::SAMPLE_RATE << Self::LOG_FREQUENCY_DIVISOR;

    /// A semitone pitch bend.
    const SEMITONE: u16 = 0x1000;

    /// Pitch bend midpoint.
    const PITCH_BEND_OFFSET: u16 = 2 * Self::SEMITONE;

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
        let pitch_bend = (Self::PITCH_BEND_OFFSET
            + ((p & ((1 << Self::LOG_NUM_BENDS) - 1)) << (12 - Self::LOG_NUM_BENDS)))
            .try_into()
            .unwrap();
        let note = wmidi::Note::try_from(
            u8::from(Self::BASE_NOTE) + u8::try_from(p >> Self::LOG_NUM_BENDS).unwrap(),
        )
        .unwrap();
        Some(Self { note, pitch_bend })
    }
}

#[cfg(test)]
mod tests {
    use super::{lookup, Midi, Note, Synth, LOG_WINDOW, MIDI_CAP, MIDPOINT};
    use core::convert::TryFrom;
    use proptest::prelude::*;
    use wmidi::{Channel, MidiMessage, PitchBend, ProgramNumber, Velocity};

    /// Smallest pitch bend.
    const PITCH_BEND_MULTIPLE: u16 = 1 << (12 - Note::LOG_NUM_BENDS);

    /// Check that converting wavelength to frequency to pitch to wavelength works.
    #[test]
    fn lookup() {
        let base_note = usize::from(u8::from(Note::BASE_NOTE)) << Note::LOG_NUM_BENDS;
        let max_note = base_note + 5 * usize::from(Note::OCTAVE);
        for &actual in &lookup::WAVELENGTHS[base_note..max_note] {
            let f = usize::from(Note::FREQUENCY_MULTIPLE / u16::try_from(actual).unwrap());
            let p = lookup::NOTES[f] + i16::try_from(base_note).unwrap();
            let expected = lookup::WAVELENGTHS[usize::try_from(p).unwrap()];
            assert_eq!(actual, expected);
        }
    }

    proptest! {
        /// Check that input is handled gracefully.
        #[test]
        fn fuzz(steps in prop::collection::vec((0_u16..(2 * MIDPOINT), prop::collection::vec(0_u8.., 0..MIDI_CAP)), 0..1000)) {
            let mut synth = Synth::default();
            for (audio_in, midi_in) in steps {
                synth.step(audio_in, &Midi::try_from(&midi_in[..]).unwrap());
            }
        }

        /// Check that converting midi to audio and back works.
        #[test]
        fn roundtrip(l: u8, h: u8, program_number in 0_u8..2, pitch_bend in 0_u16..(1 << Note::LOG_NUM_BENDS), note in 39_u8..74, velocity in 7_u8..22, off_velocity in 2_u8..0x80) {
            const MIDPOINT_OUT: u8 = 0x80;
            const OUT_BITS: u8 = 2;
            let program_number = ProgramNumber::from_u8_lossy(program_number);
            let pitch_bend = PitchBend::try_from(Note::PITCH_BEND_OFFSET + PITCH_BEND_MULTIPLE * pitch_bend).unwrap();
            let note = wmidi::Note::from_u8_lossy(note);
            let velocity = Velocity::from_u8_lossy(velocity);
            let off_velocity = Velocity::from_u8_lossy(off_velocity);
            let (mut midi2audio, mut audio2midi) = (Synth::default(), Synth::default());
            let msg = MidiMessage::ProgramChange(Channel::Ch1, program_number);
            let midi_in = Synth::serialize_midi(&msg);
            let (_, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
            prop_assert!(midi_out.is_empty());
            let msg = MidiMessage::PitchBendChange(Channel::Ch1, pitch_bend);
            let pitch_bend = Synth::serialize_midi(&msg);
            let (_, midi_out) = midi2audio.step(MIDPOINT, &pitch_bend);
            prop_assert!(midi_out.is_empty());
            let msg = MidiMessage::NoteOn(Channel::Ch1, note, velocity);
            let note_on = Synth::serialize_midi(&msg);
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &note_on);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
            prop_assert_eq!(audio_out, MIDPOINT_OUT);
            prop_assert!(midi_out.is_empty());
            let midi_in = Midi::default();
            for _ in 0..((1 << LOG_WINDOW) - 2) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                prop_assert_eq!(audio_out, MIDPOINT_OUT);
                prop_assert!(midi_out.is_empty());
            }
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
            prop_assert_eq!(audio_out, MIDPOINT_OUT);
            prop_assert_eq!(&midi_out, &pitch_bend);
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
            prop_assert_eq!(audio_out, MIDPOINT_OUT);
            prop_assert_eq!(&midi_out[0..2], &note_on[0..2]);
            let msg = MidiMessage::ChannelPressure(Channel::Ch1, velocity);
            let channel_pressure = Synth::serialize_midi(&msg);
            let msg = MidiMessage::NoteOff(Channel::Ch1, note, off_velocity);
            let note_off = Synth::serialize_midi(&msg);
            let (_, midi_out) = midi2audio.step(MIDPOINT, &note_off);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
            prop_assert_eq!(audio_out, MIDPOINT_OUT);
            prop_assert!(midi_out.is_empty());
            for _ in 0..((1 << LOG_WINDOW) - 3) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                prop_assert_eq!(audio_out, MIDPOINT_OUT);
                prop_assert!(midi_out.is_empty());
            }
            let velocity = u16::from(u8::from(velocity));
            let off_velocity = u16::from(u8::from(off_velocity));
            for _ in 0..(velocity/(off_velocity << LOG_WINDOW)) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                prop_assert_eq!(audio_out, MIDPOINT_OUT);
                prop_assert_eq!(&midi_out, &pitch_bend);
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                prop_assert_eq!(audio_out, MIDPOINT_OUT);
                prop_assert_eq!(&midi_out[0..2], &channel_pressure[0..2]);
                for _ in 0..((1 << LOG_WINDOW) - 2) {
                    let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                    prop_assert!(midi_out.is_empty());
                    let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                    prop_assert_eq!(audio_out, MIDPOINT_OUT);
                    prop_assert!(midi_out.is_empty());
                }
            }
            let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
            prop_assert!(midi_out.is_empty());
            let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
            prop_assert_eq!(audio_out, MIDPOINT_OUT);
            prop_assert_eq!(&midi_out[0..2], &note_off[0..2]);
            for _ in 0..((1 << LOG_WINDOW) - 1) {
                let (audio_out, midi_out) = midi2audio.step(MIDPOINT, &midi_in);
                prop_assert!(midi_out.is_empty());
                let (audio_out, midi_out) = audio2midi.step(u16::from(audio_out) << OUT_BITS, &midi_out);
                prop_assert_eq!(audio_out, MIDPOINT_OUT);
                prop_assert!(midi_out.is_empty());
            }
        }
    }
}
