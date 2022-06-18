//! Script to generate the lookup tables.
use arrayvec::ArrayVec;
use core::convert::TryFrom;
use synth::Note;

/// The number of distinct pitches that can be recognized.
const NUM_NOTES: usize = 0x800 << Note::LOG_FREQUENCY_DIVISOR;

/// The number of distinct wavelengths represented by midi notes.
const NUM_WAVELENGTHS: usize = 0x100 << Note::LOG_NUM_BENDS;

/// Maps a given frequency to its pitch.
#[allow(clippy::cast_possible_truncation)]
fn lookup(frequency: usize) -> i16 {
    (f64::try_from(Note::OCTAVE).unwrap()
        * (f64::try_from(i16::try_from(frequency).unwrap()).unwrap()
            / (Note::BASE_FREQUENCY * f64::try_from(1 << Note::LOG_FREQUENCY_DIVISOR).unwrap()))
        .log2())
    .round() as i16
}

/// Maps a given pitch to its wavelength.
#[allow(clippy::cast_possible_truncation)]
fn lookup_wavelength(pitch: usize) -> i16 {
    let p = i16::try_from(pitch).unwrap()
        - (i16::from(u8::from(Note::BASE_NOTE)) << Note::LOG_NUM_BENDS);
    (f64::try_from(u32::from(Note::SAMPLE_RATE) << Note::WAVELENGTH_BITS).unwrap()
        / (Note::BASE_FREQUENCY
            * (f64::try_from(p).unwrap() / f64::try_from(Note::OCTAVE).unwrap()).exp2()))
    .round() as i16
}

/// Outputs the lookup arrays to stdout.
fn main() {
    let wavelengths: ArrayVec<_, NUM_WAVELENGTHS> =
        (0..NUM_WAVELENGTHS).map(lookup_wavelength).collect();
    let wavelengths = wavelengths.into_inner().unwrap();
    println!(
        "pub const WAVELENGTHS: [i16; {}] = {:?};",
        wavelengths.len(),
        wavelengths
    );
    println!();
    let notes: ArrayVec<_, NUM_NOTES> = (0..NUM_NOTES).map(lookup).collect();
    let notes = notes.into_inner().unwrap();
    println!("pub const NOTES: [i16; {}] = {:?};", notes.len(), notes);
}
