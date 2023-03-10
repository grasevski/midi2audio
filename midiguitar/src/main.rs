//! Guitar midi controller.
#![no_main]
#![no_std]
use panic_halt as _;
use stm32l4xx_hal::{
    adc::{Event, SampleTime, Sequence, ADC},
    delay::DelayCM,
    dma::{dma1::Channels, CircBuffer, CircReadDma, DMAFrame, FrameSender},
    gpio::{Analog, PA3},
    pac::{ADC1, ADC_COMMON, DAC1, OPAMP, USART1},
    rcc::{Clocks, Enable, Rcc},
    serial::{Pins, RxDma1, Serial, TxDma1},
};
use synth::MIDI_CAP;

#[rtic::app(device = stm32l4xx_hal::pac)]
mod app {
    use super::{Audio, Midi};
    use stm32l4xx_hal::{
        pac::Peripherals,
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
        /// Audio IO.
        audio: Audio,

        /// MIDI IO.
        midi: Midi,

        /// Software digital signal processor.
        synth: Synth<'static>,
    }

    impl From<Peripherals> for Local {
        fn from(pac: Peripherals) -> Self {
            pac.VREFBUF.csr.write(|w| w.envr().set_bit());
            let mut rcc = pac.RCC.constrain();
            let mut flash = pac.FLASH.constrain();
            let mut pwr = pac.PWR.constrain(&mut rcc.apb1r1);
            let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);
            let mut gpioa = pac.GPIOA.split(&mut rcc.ahb2);
            let channels = pac.DMA1.split(&mut rcc.ahb1);
            let audio = Audio::new(
                gpioa.pa3,
                clocks,
                &mut rcc,
                pac.OPAMP,
                pac.ADC_COMMON,
                pac.ADC1,
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
            let serial = Serial::usart1(pac.USART1, (tx, rx), config, clocks, &mut rcc.apb2);
            let midi = Midi::new(serial, channels);
            let synth = Synth::default();
            Self { audio, midi, synth }
        }
    }

    /// Configures the peripherals.
    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        (Shared {}, cx.device.into(), init::Monotonics())
    }

    /// Processes the next ADC reading.
    #[task(binds = ADC1_2, local = [audio, midi, synth])]
    fn adc1_2(cx: adc1_2::Context) {
        let adc1_2::LocalResources { audio, midi, synth } = cx.local;
        let audio_in = audio.get_input();
        let mut midi_in = [0; MIDI_CAP];
        let n = midi.read(&mut midi_in[..]);
        let (audio_out, midi_out) = synth.step(audio_in, &midi_in[..n]);
        audio.set_output(audio_out);
        midi.write_slice(midi_out.as_slice());
    }
}

/// Audio IO peripherals.
pub struct Audio {
    /// Contains the most recent input sample.
    adc: ADC,

    /// Audio output destination.
    dac: DAC1,
}

impl Audio {
    /// Initializes the audio peripherals.
    fn new(
        mut pa3: PA3<Analog>,
        clocks: Clocks,
        rcc: &mut Rcc,
        opamp: OPAMP,
        adc_common: ADC_COMMON,
        adc1: ADC1,
        dac: DAC1,
    ) -> Self {
        DAC1::enable(&mut rcc.apb1r1);
        opamp.opamp1_csr.write(|w| {
            unsafe {
                w.pga_gain().bits(0b10);
                w.vm_sel().bits(0b10);
                w.opamode().bits(0b11);
            }
            w.opa_range().set_bit().opalpm().set_bit().opaen().set_bit()
        });
        OPAMP::enable(&mut rcc.apb1r1);
        adc_common.ccr.write(|w| unsafe { w.presc().bits(0b0010) });
        adc1.cfgr.write(|w| w.cont().set_bit());
        adc1.cfgr2.write(|w| {
            unsafe { w.ovsr().bits(0b001).ovss().bits(0b0110) }
                .rovse()
                .set_bit()
        });
        let mut delay = DelayCM::new(clocks);
        let mut adc = ADC::new(adc1, adc_common, &mut rcc.ahb2, &mut rcc.ccipr, &mut delay);
        adc.configure_sequence(&mut pa3, Sequence::One, SampleTime::Cycles640_5);
        adc.listen(Event::EndOfRegularSequence);
        adc.enable();
        adc.start_conversion();
        Self { adc, dac }
    }

    /// Reads the current input signal.
    fn get_input(&mut self) -> u8 {
        let ret = self.adc.get_data();
        self.adc.clear_end_flags();
        ret.try_into().unwrap()
    }

    /// Writes the output signal.
    fn set_output(&mut self, out: u8) {
        self.dac.dhr8r1.write(|w| unsafe { w.dacc1dhr().bits(out) });
    }
}

/// MIDI IO peripherals.
pub struct Midi {
    /// Midi sender.
    tx: FrameSender<&'static mut DMAFrame<MIDI_CAP>, TxDma1, MIDI_CAP>,

    /// Midi output buffer.
    buf: Option<&'static mut DMAFrame<MIDI_CAP>>,

    /// Midi input buffer.
    rx: CircBuffer<[u8; MIDI_CAP], RxDma1>,
}

impl Midi {
    /// Initializes the UART interfaces.
    fn new(serial: Serial<USART1, impl Pins<USART1>>, channels: Channels) -> Self {
        static mut RX: [u8; MIDI_CAP] = [0; MIDI_CAP];
        static mut TX: DMAFrame<MIDI_CAP> = DMAFrame::new();
        let (tx, rx) = serial.split();
        let tx = tx.with_dma(channels.4).frame_sender();
        let rx = rx.with_dma(channels.5).circ_read(unsafe { &mut RX });
        let buf = unsafe { Some(&mut TX) };
        Self { tx, buf, rx }
    }

    /// Reads the available MIDI bytes from DMA.
    fn read(&mut self, midi_in: &mut [u8]) -> usize {
        if self.buf.is_none() {
            self.buf = self.tx.transfer_complete_interrupt();
        }
        self.rx.read(midi_in).unwrap()
    }

    /// Writes the midi bytes to DMA.
    fn write_slice(&mut self, midi_out: &[u8]) {
        if midi_out.is_empty() {
            return;
        }
        let buf = self.buf.take().unwrap();
        buf.clear();
        buf.write_slice(midi_out);
        self.tx.send(buf).unwrap();
    }
}
