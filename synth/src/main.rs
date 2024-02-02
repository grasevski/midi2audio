//! Script to generate the lookup tables.
use arrayvec::ArrayVec;
use clap::Parser;
use core::convert::TryFrom;
use synth::{Note, SAMPLE_RATE};

/// The number of distinct pitches that can be recognized.
const NUM_NOTES: usize = 0x800 << Note::LOG_FREQUENCY_DIVISOR;

/// The number of distinct wavelengths represented by midi notes.
const NUM_WAVELENGTHS: usize = 0x100 << Note::LOG_NUM_BENDS;

/// Maps a given frequency to its pitch.
#[allow(clippy::cast_possible_truncation)]
fn lookup(frequency: usize) -> i8 {
    (f64::from(Note::OCTAVE)
        * (f64::from(i16::try_from(frequency).unwrap())
            / (Note::BASE_FREQUENCY * f64::from(1 << Note::LOG_FREQUENCY_DIVISOR)))
        .log2())
    .round() as i8
}

/// Maps a given pitch to its wavelength.
#[allow(clippy::cast_possible_truncation)]
fn lookup_wavelength(pitch: usize) -> i16 {
    let p = i16::try_from(pitch).unwrap()
        - (i16::from(u8::from(Note::BASE_NOTE)) << Note::LOG_NUM_BENDS);
    (f64::from(u32::from(SAMPLE_RATE) << Note::WAVELENGTH_BITS)
        / (Note::BASE_FREQUENCY * (f64::from(p) / f64::from(Note::OCTAVE)).exp2()))
    .round() as i16
}

/// Generates constant lookup tables.
#[derive(Debug, Parser)]
struct Args {
    /// Whether to generate fastf0nls C lookup tables.
    #[arg(short, long, action)]
    fastf0nls: bool,
}

/// Intermediate variables used to calculate gamma.
#[derive(Default, Clone, Copy)]
struct GammaVars {
    phi: f32,
    psi: f32,
    alpha: f32,
    lambda: f32,
    mu: f32,
}

impl GammaVars {
    /// Maximum number of harmonics.
    const MAX_MODEL_ORDER: usize = 16;

    /// Number of audio samples.
    const SAMPLE_SIZE: usize = 64;

    /// Number of points in the FFT.
    const N_FFT_GRID: usize = 2 * Self::SAMPLE_SIZE * Self::MAX_MODEL_ORDER;

    /// Number of model parameters.
    const MP: usize = Self::N_FFT_GRID / 2 + 1;

    /// Generates fastf0nls C lookup tables.
    #[allow(clippy::cast_possible_truncation)]
    fn fastf0nls() {
        static_assertions::const_assert!(GammaVars::N_FFT_GRID == 2048);
        println!("#pragma once");
        println!("#include <stdint.h>");
        const PITCH_MIN: f32 = 1.0 / (GammaVars::SAMPLE_SIZE as f32);
        println!("#define PITCH_MIN {}", PITCH_MIN);
        const PITCH_MAX: f32 = 1.0;
        println!("#define PITCH_MAX {}", PITCH_MAX);
        println!(
            "#define FFT_INIT arm_rfft_fast_init_{}_f32",
            Self::N_FFT_GRID
        );
        println!("enum {{ MAX_MODEL_ORDER = {} }};", Self::MAX_MODEL_ORDER);
        println!("enum {{ N_FFT_GRID = {} }};", Self::N_FFT_GRID);
        println!("enum {{ MP = {} }};", Self::MP);
        let min_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MIN).ceil() as usize;
        let max_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MAX).floor() as usize;
        println!(
            "enum {{ MIN_FFT_INDEX = {}, MAX_FFT_INDEX = {} }};",
            min_fft_index, max_fft_index
        );
        println!();
        let mut fft_shift_vector = [(0.0, 0.0); Self::MP];
        for (i, x) in fft_shift_vector.iter_mut().enumerate() {
            const A: f32 = core::f32::consts::PI * (GammaVars::SAMPLE_SIZE as f32 - 1.0)
                / GammaVars::N_FFT_GRID as f32;
            let v = A * i as f32;
            *x = (v.cos(), v.sin());
        }
        let fft_shift_vector: Vec<_> = fft_shift_vector
            .iter()
            .map(|(a, b)| format!("{}, {}", a, b))
            .collect();
        println!(
            "const float FFT_SHIFT_VECTOR[MP << 1] = {{\n  {},\n}};",
            fft_shift_vector.join(", ")
        );
        println!();
        const EMPTY: Vec<f32> = vec![];
        let mut cc_vectors = [EMPTY; 2 * (Self::MAX_MODEL_ORDER + 1)];
        let mut max_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MAX).floor() as usize;
        let mut n_pitches = max_fft_index - min_fft_index + 1;
        cc_vectors[0] = vec![Self::SAMPLE_SIZE as f32 * 0.5; n_pitches];
        for (k, x) in (1..).zip(&mut cc_vectors[1..]) {
            if k % 2 == 1 {
                let max_fft_index_old = max_fft_index;
                max_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MAX)
                    .min(Self::N_FFT_GRID as f32 / (k as f32 + 1.0) - 1.0)
                    as usize;
                n_pitches -= max_fft_index_old - max_fft_index;
            }
            *x = (0..n_pitches)
                .map(|i| {
                    let t = core::f32::consts::PI * (k * (i + min_fft_index)) as f32
                        / Self::N_FFT_GRID as f32;
                    0.5 * (t * Self::SAMPLE_SIZE as f32).sin() / t.sin()
                })
                .collect();
        }
        let mut max_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MAX) as usize;
        let mut n_pitches = max_fft_index - min_fft_index + 1;
        let mut n_pitches_old = 0;
        let mut gamma1 = [0.0; Self::MAX_MODEL_ORDER * Self::MP];
        let mut vars = [Self::default(); Self::MP];
        let mut gamma2 = [0.0; Self::MAX_MODEL_ORDER * Self::MP];
        let mut alpha2 = [0.0; Self::MP];
        for order in 1..=Self::MAX_MODEL_ORDER {
            let max_fft_index_old = max_fft_index;
            max_fft_index = (Self::N_FFT_GRID as f32 * PITCH_MAX)
                .min(Self::N_FFT_GRID as f32 / (2.0 * order as f32) - 1.0)
                as usize;
            let n_pitches_old_old = n_pitches_old;
            n_pitches_old = n_pitches;
            n_pitches -= max_fft_index_old - max_fft_index;
            Self::update_gamma(
                order,
                n_pitches,
                n_pitches_old,
                n_pitches_old_old,
                &mut vars,
                &mut gamma1,
                &cc_vectors[..],
            );
            Self::update_gamma_p(
                order,
                n_pitches,
                n_pitches_old,
                n_pitches_old_old,
                &mut alpha2,
                &mut gamma2,
                &cc_vectors[..],
            );
        }
        println!("const float GAMMA1[MAX_MODEL_ORDER * MP] = {{");
        for x in gamma1.chunks(Self::MP) {
            let x: Vec<_> = x.iter().map(|x| x.to_string()).collect();
            println!("  {},", x.join(", "));
        }
        println!("}};");
        println!();
        println!("const float GAMMA2[MAX_MODEL_ORDER * MP] = {{");
        for x in gamma2.chunks(Self::MP) {
            let x: Vec<_> = x.iter().map(|x| x.to_string()).collect();
            println!("  {},", x.join(", "));
        }
        println!("}};");
        println!();
        println!("const float *CC_VECTORS[(MAX_MODEL_ORDER + 1) << 1] = {{");
        for x in cc_vectors {
            let x: Vec<_> = x.iter().map(|f| f.to_string()).collect();
            println!("  (const float[]){{{}}},", x.join(", "));
        }
        println!("}};");
    }

    /// Does an in place update to gamma for the given order.
    fn update_gamma(
        order: usize,
        n_pitches: usize,
        n_pitches_old: usize,
        n_pitches_old_old: usize,
        vars: &mut [Self],
        gamma: &mut [f32],
        cc_vectors: &[Vec<f32>],
    ) {
        let (tf, hf) = (&cc_vectors, &cc_vectors[2..]);
        if order == 1 {
            for (i, v) in vars[..n_pitches].iter_mut().enumerate() {
                gamma[i] = 1.0 / (tf[0][i] + hf[0][i]);
                v.psi = gamma[i];
                v.phi = gamma[i] * tf[1][i];
            }
            return;
        } else if order == 2 {
            for (i, v) in vars[..n_pitches].iter_mut().enumerate() {
                let t1 = tf[0][i] + hf[0][i];
                let t2 = tf[1][i] + hf[1][i];
                let t4 = 1.0 / (t1 * (tf[0][i] + hf[2][i]) - t2 * t2);
                v.alpha = t2 * gamma[i];
                gamma[Self::MP + i] = -t2 * t4;
                gamma[Self::MP + n_pitches + i] = t1 * t4;
            }
            return;
        }
        for (i, v) in vars[..n_pitches].iter_mut().enumerate() {
            v.lambda = tf[order - 1][i] + hf[order - 3][i];
            v.mu = 0.0;
        }
        for k in 0..order - 2 {
            for i in 0..n_pitches {
                let t = tf[order - 2 - k][i] + hf[k + order - 2][i];
                let v2 = vars[k * n_pitches_old + i];
                let v = &mut vars[i];
                v.lambda -= v2.psi * t;
                v.mu -= v2.phi * t;
            }
        }
        let t = vars.to_vec();
        for k in 0..order - 2 {
            for i in 0..n_pitches {
                let v = vars[i];
                let v2 = &mut vars[n_pitches * k + i];
                let t = t[n_pitches_old * k + i];
                let g = gamma[Self::MP * (order - 2) + n_pitches_old * k + i];
                v2.phi = t.phi + v.lambda * g;
                v2.psi = t.psi + v.mu * g;
            }
        }
        for i in 0..n_pitches {
            let v = vars[i];
            let v2 = &mut vars[n_pitches * (order - 2) + i];
            let g = gamma[(Self::MP + n_pitches_old) * (order - 2) + i];
            v2.phi = v.lambda * g;
            v2.psi = v.mu * g;
        }
        let mut t = vec![0.0; n_pitches];
        for k in 0..order - 1 {
            for (i, x) in t.iter_mut().enumerate() {
                let t2 = tf[order - 1 - k][i] + hf[order - 1 + k][i];
                *x += t2 * gamma[Self::MP * (order - 2) + n_pitches_old * k + i];
            }
        }
        let mut t2 = vec![0.0; n_pitches];
        for (i, v) in vars[..n_pitches].iter_mut().enumerate() {
            t2[i] = v.alpha - t[i];
            v.alpha = t[i];
        }
        let mut t3 = vec![0.0; n_pitches * (order - 1)];
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                t3[k * n_pitches + i] =
                    t2[i] * gamma[Self::MP * (order - 2) + n_pitches_old * k + i];
                if k != 0 {
                    t3[k * n_pitches + i] +=
                        gamma[Self::MP * (order - 2) + n_pitches_old * (k - 1) + i];
                }
                if k != order - 2 {
                    t3[k * n_pitches + i] +=
                        gamma[Self::MP * (order - 2) + n_pitches_old * (k + 1) + i];
                    t3[k * n_pitches + i] -=
                        gamma[Self::MP * (order - 3) + n_pitches_old_old * k + i];
                }
                let t1 = vars[n_pitches * (order - 2) + i].psi * vars[n_pitches * k + i].phi;
                t3[k * n_pitches + i] += t1;
                let t1 = vars[n_pitches * (order - 2) + i].phi * vars[n_pitches * k + i].psi;
                t3[k * n_pitches + i] -= t1;
            }
        }
        for x in &mut t {
            *x = 0.0;
        }
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                t[i] += (tf[order - 1 - k][i] + hf[order - 1 + k][i]) * t3[k * n_pitches + i];
            }
        }
        for i in 0..n_pitches {
            let mut t2 = t[i] / gamma[(Self::MP + n_pitches_old) * (order - 2) + i];
            t2 += tf[0][i] + hf[2 * (order - 1)][i];
            gamma[(Self::MP + n_pitches) * (order - 1) + i] = 1.0 / t2;
        }
        let mut t4 = vec![0.0; n_pitches];
        for (i, x) in t4.iter_mut().enumerate() {
            *x = gamma[(Self::MP + n_pitches) * (order - 1) + i]
                / gamma[(Self::MP + n_pitches_old) * (order - 2) + i];
        }
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                gamma[Self::MP * (order - 1) + n_pitches * k + i] = t3[n_pitches * k + i] * t4[i];
            }
        }
    }

    /// Does an in place update to gamma p for the given order.
    fn update_gamma_p(
        order: usize,
        n_pitches: usize,
        n_pitches_old: usize,
        n_pitches_old_old: usize,
        alpha: &mut [f32],
        gamma: &mut [f32],
        cc_vectors: &[Vec<f32>],
    ) {
        let (tf, hf) = (&cc_vectors, &cc_vectors[2..]);
        if order == 1 {
            for (i, g) in gamma[..n_pitches].iter_mut().enumerate() {
                *g = 1.0 / (tf[0][i] + hf[0][i]);
            }
            return;
        } else if order == 2 {
            for (i, a) in alpha[..n_pitches].iter_mut().enumerate() {
                let t1 = tf[0][i] - hf[0][i];
                let t2 = tf[1][i] - hf[1][i];
                let t4 = 1.0 / (t1 * (tf[0][i] - hf[2][i]) - t2 * t2);
                *a = t2 * gamma[i];
                gamma[Self::MP + i] = -t2 * t4;
                gamma[Self::MP + n_pitches + i] = t1 * t4;
            }
            return;
        }
        let mut t = vec![0.0; n_pitches];
        for k in 0..order - 1 {
            for (i, x) in t.iter_mut().enumerate() {
                let t2 = tf[order - 1 - k][i] + hf[order - 1 + k][i];
                *x += t2 * gamma[Self::MP * (order - 2) + n_pitches_old * k];
            }
        }
        let mut t2 = vec![0.0; n_pitches];
        for (i, a) in alpha[..n_pitches].iter_mut().enumerate() {
            t2[i] = *a - t[i];
            *a = t[i];
        }
        let mut t3 = vec![0.0; n_pitches * (order - 1)];
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                t3[k * n_pitches + i] =
                    t2[i] * gamma[Self::MP * (order - 2) + n_pitches_old * k + i];
                if k != 0 {
                    t3[k * n_pitches + i] +=
                        gamma[Self::MP * (order - 2) + n_pitches_old * (k - 1) + i];
                }
                if k != order - 2 {
                    t3[k * n_pitches + i] +=
                        gamma[Self::MP * (order - 2) + n_pitches_old * (k + 1) + i];
                    t3[k * n_pitches + i] -=
                        gamma[Self::MP * (order - 3) + n_pitches_old_old * k + i];
                }
            }
        }
        for x in &mut t {
            *x = 0.0;
        }
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                t[i] += (tf[order - 1 - k][i] - hf[order - 1 + k][i]) * t3[k * n_pitches + i];
            }
        }
        for i in 0..n_pitches {
            let mut t2 = t[i] / gamma[(Self::MP + n_pitches_old) * (order - 2) + i];
            t2 += tf[0][i] - hf[2 * (order - 1)][i];
            gamma[(Self::MP + n_pitches) * (order - 1) + i] = 1.0 / t2;
        }
        let mut t4 = vec![0.0; n_pitches];
        for (i, x) in t4.iter_mut().enumerate() {
            *x = gamma[(Self::MP + n_pitches) * (order - 1) + i]
                / gamma[(Self::MP + n_pitches_old) * (order - 2) + i];
        }
        for k in 0..order - 1 {
            for i in 0..n_pitches {
                gamma[Self::MP * (order - 1) + n_pitches * k + i] = t3[n_pitches * k + i] * t4[i];
            }
        }
    }
}

/// Outputs the lookup arrays to stdout.
fn main() {
    if Args::parse().fastf0nls {
        GammaVars::fastf0nls();
        return;
    }
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
    println!("pub const NOTES: [i8; {}] = {:?};", notes.len(), notes);
}
