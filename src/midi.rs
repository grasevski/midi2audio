use arrayvec::ArrayVec;
use core::mem::size_of;

const BUFSIZ: usize = 4;

/// Buffer for reading or writing midi messages.
pub type Buf = ArrayVec<u8, BUFSIZ>;

#[derive(Clone, Copy)]
pub enum Status {
    RealTimeTimingClock = 0xf8,
    RealTimeUndefined1 = 0xf9,
    RealTimeStart = 0xfa,
    RealTimeContinue = 0xfb,
    RealTimeStop = 0xfc,
    RealTimeUndefined2 = 0xfd,
    RealTimeActiveSensing = 0xfe,
    RealTimeSystemReset = 0xff,
    Sox = 0xf0,
    Eox = 0xf7,
}

impl Status {
    fn len(status: u8) -> usize {
        0
    }
}

pub struct Midi {
    pub status: u8,
    pub data: [u8; 2],
}

const SIZE: usize = size_of::<Midi>();

pub type Msg = ArrayVec<u8, SIZE>;

impl From<Midi> for Msg {
    fn from(msg: Midi) -> Msg {
        let mut out = Msg::default();
        out.push(msg.status);
        out.try_extend_from_slice(&msg.data[0..Status::len(msg.status)])
            .unwrap();
        out
    }
}

impl From<u8> for Midi {
    fn from(status: u8) -> Self {
        Self {
            status,
            data: Default::default(),
        }
    }
}

#[derive(Default)]
pub struct Parser {
    msg: Option<Midi>,
    remaining: usize,
}

impl Parser {
    pub fn parse(&mut self, data: &Buf) -> ArrayVec<Midi, BUFSIZ> {
        let mut ret = ArrayVec::default();
        for &byte in data {
            if let Some(ref mut msg) = self.msg {
                let n = Status::len(msg.status);
                msg.data[n - self.remaining] = byte;
                self.remaining -= 1;
            } else {
                let msg = Midi::from(byte);
                self.remaining = Status::len(msg.status);
                self.msg = Some(msg);
            }
            if self.remaining > 0 {
            } else if let Some(msg) = self.msg.take() {
                ret.push(msg);
            }
        }
        ret
    }
}
