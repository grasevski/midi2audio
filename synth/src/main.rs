use synth::{lookup_notes, lookup_wavelengths};

fn main() {
    let (wavelengths, notes) = (lookup_wavelengths(), lookup_notes());
    println!(
        "pub const WAVELENGTHS: [i16; {}] = {:?};",
        wavelengths.len(),
        wavelengths
    );
    println!();
    println!("pub const NOTES: [i16; {}] = {:?};", notes.len(), notes);
}
