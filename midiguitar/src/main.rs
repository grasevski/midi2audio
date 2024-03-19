//! Guitar midi controller.
#![no_main]
#![no_std]
use core::cmp::Ordering;
use defmt_rtt as _;
use panic_probe as _;
use stm32l4::stm32l4x2::{ADC1, ADC_COMMON, DAC, DMA1, OPAMP, TIM6, USART1};
use synth::WINDOW;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[rtic::app(device = stm32l4::stm32l4x2)]
mod app {
    use super::{wait, Audio, Midi};
    use synth::{Synth, MIDI_CAP, WINDOW};

    /// Shared resources.
    #[shared]
    struct Shared {}

    /// Uniquely owned resources.
    #[local]
    struct Local {
        /// Audio IO.
        audio: Audio,

        /// MIDI IO.
        midi: Midi,

        /// Software digital signal processor.
        synth: Synth,
    }

    /// Configures the peripherals.
    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        const RCC_WAIT: u32 = 2;
        const N: u8 = WINDOW << 1;
        static mut INPUT: [u8; N as usize] = [0; N as usize];
        static mut OUTPUT: [u8; N as usize] = [0; N as usize];
        static mut RX: [u8; MIDI_CAP] = [0; MIDI_CAP];
        static mut TX: [u8; MIDI_CAP] = [0; MIDI_CAP];
        let pac = cx.device;
        let flash = pac.FLASH;
        flash.acr.modify(|_, w| w.prften().set_bit());
        let rcc = pac.RCC;
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());
        wait(RCC_WAIT);
        rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());
        wait(RCC_WAIT);
        let pwr = pac.PWR;
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
        while pwr.cr1.read().dbp().bit_is_clear() {}
        rcc.bdcr.modify(|_, w| w.lseon().set_bit());
        while rcc.bdcr.read().lserdy().bit_is_clear() {}
        rcc.apb1enr1.modify(|_, w| w.pwren().clear_bit());
        rcc.cr.modify(|_, w| w.pllon().clear_bit());
        while rcc.cr.read().pllrdy().bit_is_set() {}
        rcc.pllcfgr
            .modify(|_, w| unsafe { w.pllsrc().bits(1).plln().bits(40) });
        rcc.cr.modify(|_, w| w.pllon().set_bit());
        rcc.pllcfgr.modify(|_, w| w.pllren().set_bit());
        while rcc.cr.read().pllrdy().bit_is_clear() {}
        flash.acr.modify(|_, w| unsafe { w.latency().bits(4) });
        rcc.cfgr
            .modify(|_, w| unsafe { w.sw().bits(3).ppre2().bits(7) });
        while rcc.cfgr.read().sws().bits() != 3 {}
        rcc.cr.modify(|_, w| w.msipllen().set_bit());
        rcc.ccipr.modify(|_, w| w.adcsel().bits(3));

        rcc.apb1enr1
            .modify(|_, w| w.opampen().set_bit().dac1en().set_bit().tim6en().set_bit());
        wait(RCC_WAIT);
        rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
        wait(RCC_WAIT);
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
        wait(RCC_WAIT);
        rcc.ahb2enr.modify(|_, w| {
            w.gpioaen().set_bit().gpioben().set_bit();
            w.gpiocen().set_bit().gpiohen().set_bit();
            w.adcen().set_bit()
        });
        wait(RCC_WAIT);

        let gpioa = pac.GPIOA;
        gpioa.afrh.modify(|_, w| w.afrh9().bits(7).afrh10().bits(7));
        gpioa
            .moder
            .modify(|_, w| w.moder9().bits(2).moder10().bits(2));

        let (adc1, dac, usart1, dma1) = (pac.ADC1, pac.DAC, pac.USART1, pac.DMA1);

        dma1.cselr
            .modify(|_, w| w.c3s().bits(6).c4s().bits(2).c5s().bits(2));

        dma1.cndtr1.modify(|_, w| w.ndt().bits(N.into()));
        dma1.cpar1
            .modify(|_, w| unsafe { w.pa().bits(adc1.dr.as_ptr() as u32) });
        dma1.cmar1
            .modify(|_, w| unsafe { w.ma().bits(&INPUT as *const _ as u32) });
        dma1.ccr1.modify(|_, w| {
            unsafe { w.psize().bits(1) };
            w.minc().set_bit().circ().set_bit();
            w.htie().set_bit().tcie().set_bit()
        });
        dma1.ccr1.modify(|_, w| w.en().set_bit());

        dma1.cndtr3.modify(|_, w| w.ndt().bits(N.into()));
        dma1.cpar3
            .modify(|_, w| unsafe { w.pa().bits(dac.dhr8r1.as_ptr() as u32) });
        dma1.cmar3
            .modify(|_, w| unsafe { w.ma().bits(&OUTPUT as *const _ as u32) });
        dma1.ccr3.modify(|_, w| {
            unsafe { w.psize().bits(1) };
            w.minc().set_bit().circ().set_bit().dir().set_bit()
        });
        dma1.ccr3.modify(|_, w| w.en().set_bit());

        dma1.cpar4
            .modify(|_, w| unsafe { w.pa().bits(usart1.tdr.as_ptr() as u32) });
        dma1.cmar4
            .modify(|_, w| unsafe { w.ma().bits(&TX as *const _ as u32) });
        dma1.ccr4.modify(|_, w| w.minc().set_bit().dir().set_bit());

        dma1.cndtr5.modify(|_, w| w.ndt().bits(MIDI_CAP as u16));
        dma1.cpar5
            .modify(|_, w| unsafe { w.pa().bits(usart1.rdr.as_ptr() as u32) });
        dma1.cmar5
            .modify(|_, w| unsafe { w.ma().bits(&RX as *const _ as u32) });
        dma1.ccr5.modify(|_, w| w.minc().set_bit().circ().set_bit());
        dma1.ccr5.modify(|_, w| w.en().set_bit());

        let audio = unsafe {
            Audio::new(
                &INPUT,
                &mut OUTPUT,
                pac.OPAMP,
                (adc1, pac.ADC_COMMON),
                dac,
                pac.TIM6,
            )
        };
        let midi = unsafe { Midi::new(&RX, &mut TX, dma1, usart1) };
        let synth = Default::default();
        (Shared {}, Local { audio, midi, synth }, init::Monotonics())
    }

    /// Processes the next ADC reading.
    #[task(binds = DMA1_CH1, local = [audio, midi, synth])]
    fn dma1_ch1(cx: dma1_ch1::Context) {
        let dma1_ch1::LocalResources { audio, midi, synth } = cx.local;
        let (mut midi_in, mut midi_out) = ([0; MIDI_CAP], synth::Midi::new_const());
        let n = midi.read(&mut midi_in[..]);
        let ((audio_in, audio_out), mut k) = (audio.get(), n);
        for (x, y) in audio_in.iter().cloned().zip(audio_out.iter_mut()) {
            let (o, mo) = synth.step(x, &midi_in[..k]);
            *y = o;
            midi_out.try_extend_from_slice(&mo[..]).unwrap();
            k = 0;
        }
        midi.write_slice(&midi_out[..]);
        defmt::println!(
            "mi={} mo={} i={} o={}",
            &midi_in[..n],
            &midi_out[..],
            audio_in,
            audio_out
        );
    }
}

/// Busy loops for the given amount of cpu cycles.
fn wait(t: u32) {
    for _ in 0..t {
        core::hint::black_box(());
    }
}

/// Audio IO peripherals.
pub struct Audio {
    /// Buffer position.
    k: bool,

    /// ADC buffer.
    rx: &'static [u8],

    /// DAC buffer.
    tx: &'static mut [u8],
}

impl Audio {
    /// Initializes the audio peripherals.
    fn new(
        rx: &'static [u8],
        tx: &'static mut [u8],
        opamp: OPAMP,
        adc: (ADC1, ADC_COMMON),
        dac: DAC,
        tim6: TIM6,
    ) -> Self {
        const MILLISECOND: u32 = 80000;

        opamp.opamp1_csr.modify(|_, w| {
            unsafe { w.pga_gain().bits(3).vm_sel().bits(2) };
            w.usertrim().set_bit().opalpm().set_bit()
        });
        opamp
            .opamp1_lpotr
            .modify(|_, w| w.trimlpoffsetn().bits(31).trimlpoffsetp().bits(31));
        opamp.opamp1_csr.modify(|_, w| w.calon().set_bit());
        opamp.opamp1_csr.modify(|_, w| w.opaen().set_bit());
        let (mut trim, mut delta) = (16, 8);
        while delta != 0 {
            opamp
                .opamp1_lpotr
                .modify(|_, w| w.trimlpoffsetn().bits(trim));
            wait(MILLISECOND);
            if opamp.opamp1_csr.read().calout().bit_is_set() {
                trim -= delta;
            } else {
                trim += delta;
            }
            delta >>= 1;
        }
        opamp
            .opamp1_lpotr
            .modify(|_, w| w.trimlpoffsetn().bits(trim));
        wait(MILLISECOND);
        if opamp.opamp1_csr.read().calout().bit_is_clear() {
            trim += 1;
            opamp
                .opamp1_lpotr
                .modify(|_, w| w.trimlpoffsetn().bits(trim));
        }
        opamp.opamp1_csr.modify(|_, w| w.calsel().set_bit());
        trim = 16;
        delta = 8;
        while delta != 0 {
            opamp
                .opamp1_lpotr
                .modify(|_, w| w.trimlpoffsetp().bits(trim));
            wait(MILLISECOND);
            if opamp.opamp1_csr.read().calout().bit_is_set() {
                trim += delta;
            } else {
                trim -= delta;
            }
            delta >>= 1;
        }
        opamp
            .opamp1_lpotr
            .modify(|_, w| w.trimlpoffsetp().bits(trim));
        wait(MILLISECOND);
        if opamp.opamp1_csr.read().calout().bit_is_set() {
            trim += 1;
            opamp
                .opamp1_lpotr
                .modify(|_, w| w.trimlpoffsetp().bits(trim));
        }
        opamp.opamp1_csr.modify(|_, w| w.opaen().clear_bit());
        opamp.opamp1_csr.modify(|_, w| w.calon().clear_bit());
        opamp
            .opamp1_csr
            .modify(|_, w| unsafe { w.opamode().bits(2) });
        opamp.opamp1_csr.modify(|_, w| w.opaen().set_bit());

        let (adc1, adc_common) = adc;
        adc1.cr.modify(|_, w| w.deeppwd().clear_bit());
        adc1.cr.modify(|_, w| w.advregen().set_bit());
        wait(2000);
        adc1.cr.modify(|_, w| w.adcal().set_bit());
        while adc1.cr.read().adcal().bit_is_set() {}
        adc_common.ccr.modify(|_, w| unsafe { w.presc().bits(10) });
        adc1.cfgr.modify(|_, w| {
            unsafe { w.exten().bits(1).extsel().bits(13).res().bits(2) };
            w.dmacfg().set_bit().dmaen().set_bit()
        });
        adc1.smpr1.modify(|_, w| unsafe { w.smp1().bits(7) });
        adc1.sqr1.modify(|_, w| unsafe { w.sq1().bits(8) });
        adc1.cr.modify(|_, w| w.aden().set_bit());
        while adc1.isr.read().adrdy().bit_is_clear() {}
        adc1.cr.modify(|_, w| w.adstart().set_bit());

        dac.cr.modify(|_, w| w.ten1().set_bit());
        dac.cr.modify(|_, w| w.dmaen1().set_bit());
        dac.cr.modify(|_, w| w.en1().set_bit());

        tim6.arr.modify(|_, w| w.arr().bits(1));
        tim6.psc.modify(|_, w| w.psc().bits(15624));
        tim6.egr.write(|w| w.ug().set_bit());
        tim6.cr2.modify(|_, w| unsafe { w.mms().bits(2) });
        tim6.cr1.modify(|_, w| w.cen().set_bit());

        Self { k: false, rx, tx }
    }

    /// Gets audio IO buffers.
    fn get(&mut self) -> (&[u8], &mut [u8]) {
        let ix = WINDOW.into();
        let r = if self.k {
            0..ix
        } else {
            ix..(WINDOW << 1).into()
        };
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

    /// DMA peripheral registers.
    dma1: DMA1,
}

impl Midi {
    /// Initializes the UART interfaces.
    fn new(rx: &'static [u8], tx: &'static mut [u8], dma1: DMA1, usart1: USART1) -> Self {
        usart1.cr1.modify(|_, w| w.te().set_bit().re().set_bit());
        usart1.brr.modify(|_, w| w.brr().bits(2560));
        usart1.cr1.modify(|_, w| w.ue().set_bit());
        while usart1.isr.read().teack().bit_is_clear() {}
        while usart1.isr.read().reack().bit_is_clear() {}
        usart1
            .cr3
            .modify(|_, w| w.dmar().set_bit().dmat().set_bit());
        Self { k: 0, rx, tx, dma1 }
    }

    /// Reads the available MIDI bytes from DMA.
    fn read(&mut self, midi_in: &mut [u8]) -> usize {
        let k = self.rx.len() - usize::from(self.dma1.cndtr5.read().ndt().bits());
        let ret = match self.k.cmp(&k) {
            Ordering::Less => {
                let o = k - self.k;
                midi_in[..o].clone_from_slice(&self.rx[k..self.k]);
                o
            }
            Ordering::Greater => {
                let o = self.rx.len() - self.k;
                midi_in[..o].clone_from_slice(&self.rx[self.k..]);
                let ret = o + k;
                midi_in[o..ret].clone_from_slice(&self.rx[..k]);
                ret
            }
            Ordering::Equal => 0,
        };
        self.k = k;
        ret
    }

    /// Writes the midi bytes to DMA.
    fn write_slice(&mut self, midi_out: &[u8]) {
        let n = midi_out.len().try_into().unwrap();
        self.tx[..midi_out.len()].clone_from_slice(midi_out);
        self.dma1.ccr4.modify(|_, w| w.en().clear_bit());
        self.dma1.cndtr4.modify(|_, w| w.ndt().bits(n));
        self.dma1.ccr4.modify(|_, w| w.en().set_bit());
    }
}
