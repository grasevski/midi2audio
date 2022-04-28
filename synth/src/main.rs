//! Script to generate the lookup tables.
use arrayvec::ArrayVec;
use core::convert::TryFrom;
use synth::Note;

/// The number of distinct pitches that can be recognized.
const NUM_NOTES: usize = 8192;

/// The number of distinct wavelengths represented by midi notes.
const NUM_WAVELENGTHS: usize = 2048;

/// The number of subdivisions per octave.
const DENOMINATOR: f64 = (12 << Note::LOG_NUM_BENDS) as f64;

/// Frequency of A.
const BASE_FREQUENCY: f64 = 110.0;

/// Maps a given frequency to its pitch.
fn lookup(frequency: usize) -> i16 {
    (DENOMINATOR
        * (f64::try_from(i16::try_from(frequency).unwrap()).unwrap() / BASE_FREQUENCY).log2())
    .round() as i16
}

/// Maps a given pitch to its wavelength.
fn lookup_wavelength(pitch: usize) -> i16 {
    let p = i16::try_from(pitch).unwrap()
        - (i16::from(u8::from(Note::BASE_NOTE)) << Note::LOG_NUM_BENDS);
    (8192.0 / (BASE_FREQUENCY * (f64::try_from(p).unwrap() / DENOMINATOR).exp2())).round() as i16
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
