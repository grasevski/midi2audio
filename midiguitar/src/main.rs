//! Guitar midi controller.
#![no_main]
#![no_std]
use core::cmp::Ordering;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4xx_hal::{
    adc::{Resolution, SampleTime, Sequence, ADC},
    delay::DelayCM,
    gpio::{Analog, PA3},
    pac::{ADC1, ADC_COMMON, DAC1, DMA1, OPAMP, TIM6, USART1},
    prelude::*,
    rcc::{Clocks, Enable as _, Rcc},
    timer::Timer,
};
use synth::{MIDI_CAP, SAMPLE_RATE, WINDOW};

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[rtic::app(device = stm32l4xx_hal::pac)]
mod app {
    use super::{Audio, Midi};
    use stm32l4xx_hal::{
        pac::{DMA1, Peripherals},
        prelude::*,
        serial::{Config, Serial},
    };
    use synth::{Synth, MIDI_CAP};

    /// Shared resources.
    #[shared]
    struct Shared {}

    /// Uniquely owned resources.
    #[local]
    struct Local {
        /// Memory buffers used by audio and MIDI.
        dma: DMA1,

        /// Audio IO.
        audio: Audio,

        /// MIDI IO.
        midi: Midi,

        /// Software digital signal processor.
        synth: Synth,
    }

    impl From<Peripherals> for Local {
        fn from(pac: Peripherals) -> Self {
            pac.VREFBUF.csr.write(|w| w.envr().set_bit());
            let (mut rcc, mut flash) = (pac.RCC.constrain(), pac.FLASH.constrain());
            let mut pwr = pac.PWR.constrain(&mut rcc.apb1r1);
            let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
            let (mut gpioa, dma) = (pac.GPIOA.split(&mut rcc.ahb2), pac.DMA1);
            let audio = Audio::new(
                gpioa.pa3,
                clocks,
                &mut rcc,
                &dma,
                pac.TIM6,
                (pac.OPAMP, pac.ADC_COMMON, pac.ADC1),
                pac.DAC1,
            );
            let tx = gpioa
                .pa9
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let rx =
                gpioa
                    .pa10
                    .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let config = Config::default().baudrate(31250.bps());
            Serial::usart1(pac.USART1, (tx, rx), config, clocks, &mut rcc.apb2);
            let (midi, synth) = (Midi::new(&dma), Synth::default());
            Self { dma, audio, midi, synth }
        }
    }

    /// Configures the peripherals.
    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        (Shared {}, cx.device.into(), init::Monotonics())
    }

    /// Processes the next ADC reading.
    #[task(binds = DMA1_CH1, local = [dma, audio, midi, synth])]
    fn dma1_channel1(cx: dma1_channel1::Context) {
        let dma1_channel1::LocalResources { dma, audio, midi, synth } = cx.local;
        let (mut midi_in, mut midi_out) = ([0; MIDI_CAP], synth::Midi::new_const());
        let n = midi.read(dma, &mut midi_in[..]);
        let ((audio_in, audio_out), mut k) = (audio.get(dma), n);
        for (x, y) in audio_in.iter().cloned().zip(audio_out.iter_mut()) {
            let (o, mo) = synth.step(x.try_into().unwrap(), &midi_in[..k]);
            *y = o;
            midi_out.try_extend_from_slice(&mo[..]).unwrap();
            k = 0;
        }
        midi.write_slice(dma, &midi_out[..]);
        defmt::println!(
            "mi={} mo={} i={} o={}",
            &midi_in[..n],
            &midi_out[..],
            audio_in,
            audio_out
        );
    }
}

/// Audio IO peripherals.
pub struct Audio {
    /// Buffer position.
    k: bool,

    /// ADC buffer.
    rx: &'static [u16],

    /// DAC buffer.
    tx: &'static mut [u8],
}

impl Audio {
    /// Initializes the audio peripherals.
    fn new(
        mut pa3: PA3<Analog>,
        clocks: Clocks,
        rcc: &mut Rcc,
        dma: &DMA1,
        tim: TIM6,
        input: (OPAMP, ADC_COMMON, ADC1),
        dac: DAC1,
    ) -> Self {
        const N: u16 = (WINDOW << 1) as u16;
        static mut RX: [u16; N as usize] = [0; N as usize];
        static mut TX: [u8; N as usize] = [0; N as usize];
        let (opamp, adc_common, adc) = input;
        DAC1::enable(&mut rcc.apb1r1);
        dac.shhr.write(|w| unsafe { w.thold1().bits(0x1ff) });
        dac.mcr.write(|w| unsafe { w.mode1().bits(0b100) });
        dac.cr
            .write(|w| w.en1().set_bit().dmaen1().set_bit().ten1().set_bit());
        OPAMP::enable(&mut rcc.apb1r1);
        opamp.opamp1_csr.write(|w| {
            unsafe {
                w.pga_gain().bits(0b10);
                w.vm_sel().bits(0b10);
                w.opamode().bits(0b10);
            }
            w.opa_range().set_bit().opalpm().set_bit().opaen().set_bit()
        });
        let mut delay = DelayCM::new(clocks);
        adc_common.ccr.write(|w| unsafe { w.presc().bits(0b1011) });
        adc.cfgr.write(|w| {
            w.dmacfg().set_bit().dmaen().set_bit();
            unsafe { w.exten().bits(0b01).extsel().bits(0b1101) }
        });
        let mut adc = ADC::new(adc, adc_common, &mut rcc.ahb2, &mut rcc.ccipr, &mut delay);
        adc.set_resolution(Resolution::Bits8);
        adc.configure_sequence(&mut pa3, Sequence::One, SampleTime::Cycles47_5);

        dma.ccr1.write(|w| {
            w.circ().set_bit().en().set_bit().minc().set_bit();
            w.msize().bits16().psize().bits16().htie().set_bit().tcie().set_bit()
        });
        dma.cndtr1.write(|w| w.ndt().bits(N));
        dma.cpar1
            .write(|w| unsafe { w.bits(&(*ADC1::ptr()).dr as *const _ as u32) });
        dma.cmar1
            .write(|w| unsafe { w.bits(&RX as *const _ as u32) });

        dma.ccr3.write(|w| {
            w.circ().set_bit().en().set_bit().minc().set_bit();
            w.msize().bits8().psize().bits8().dir().set_bit()
        });
        dma.cndtr3.write(|w| w.ndt().bits(N));
        dma.cpar3
            .write(|w| unsafe { w.bits(&(*DAC1::ptr()).dhr8r1 as *const _ as u32) });
        dma.cmar3
            .write(|w| unsafe { w.bits(&TX as *const _ as u32) });

        dma.cselr.write(|w| w.c3s().map6());

        Timer::free_running_tim6(
            tim,
            clocks,
            u32::from(SAMPLE_RATE).Hz(),
            true,
            &mut rcc.apb1r1,
        );
        let (k, (rx, tx)) = (false, unsafe { (&RX, &mut TX) });
        Self { k, rx, tx }
    }

    /// Gets audio IO buffers.
    fn get(&mut self, dma: &DMA1) -> (&[u16], &mut [u8]) {
        dma.ifcr.write(|w| w.cgif1().set_bit());
        let ix = WINDOW.into();
        let r = if self.k { 0..ix } else { ix..(WINDOW << 1).into() };
        self.k = !self.k;
        (&self.rx[r.clone()], &mut self.tx[r])
    }
}

/// MIDI IO peripherals.
pub struct Midi {
    /// Midi input buffer position.
    k: usize,

    /// Midi input buffer.
    rx: &'static [u8],

    /// Midi output buffer.
    tx: &'static mut [u8],
}

impl Midi {
    /// Initializes the UART interfaces.
    fn new(dma: &DMA1) -> Self {
        static mut RX: [u8; MIDI_CAP] = [0; MIDI_CAP];
        static mut TX: [u8; MIDI_CAP] = [0; MIDI_CAP];

        dma.ccr5.write(|w| {
            w.circ().set_bit().en().set_bit().minc().set_bit();
            w.msize().bits8().psize().bits8()
        });
        dma.cndtr5.write(|w| w.ndt().bits(MIDI_CAP.try_into().unwrap()));
        dma.cpar5
            .write(|w| unsafe { w.bits(&(*USART1::ptr()).rdr as *const _ as u32) });
        dma.cmar5
            .write(|w| unsafe { w.bits(&RX as *const _ as u32) });
        dma.cselr.write(|w| w.c5s().map2());

        dma.ccr4.write(|w| {
            w.minc().set_bit().msize().bits8().psize().bits8().dir().set_bit()
        });
        dma.cpar4
            .write(|w| unsafe { w.bits(&(*USART1::ptr()).tdr as *const _ as u32) });
        dma.cmar4
            .write(|w| unsafe { w.bits(&TX as *const _ as u32) });
        dma.cselr.write(|w| w.c4s().map2());

        let (k, (rx, tx)) = (0, unsafe { (&RX, &mut TX) });
        Self { k, rx, tx }
    }

    /// Reads the available MIDI bytes from DMA.
    fn read(&mut self, dma: &DMA1, midi_in: &mut [u8]) -> usize {
        let k = self.rx.len() - usize::from(dma.cndtr5.read().ndt().bits());
        let ret = match self.k.cmp(&k) {
            Ordering::Less => {
                let o = k - self.k;
                midi_in[..o].clone_from_slice(&self.rx[k..self.k]);
                o
            },
            Ordering::Greater => {
                let o = self.rx.len() - self.k;
                midi_in[..o].clone_from_slice(&self.rx[self.k..]);
                let ret = o + k;
                midi_in[o..ret].clone_from_slice(&self.rx[..k]);
                ret
            },
            Ordering::Equal => 0,
        };
        self.k = k;
        ret
    }

    /// Writes the midi bytes to DMA.
    fn write_slice(&mut self, dma: &DMA1, midi_out: &[u8]) {
        self.tx[..midi_out.len()].clone_from_slice(midi_out);
        dma.cndtr4.write(|w| w.ndt().bits(midi_out.len().try_into().unwrap()));
    }
}
