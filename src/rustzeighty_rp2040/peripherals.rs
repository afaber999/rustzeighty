

#[derive(Default)]
pub struct Peripherals {  
}

impl Peripherals {
    pub fn new() -> Self {
        Self {

        }
    }    
}

impl rz80::Bus for Peripherals {
    fn cpu_inp(&mut self, _port: rz80::RegT) -> rz80::RegT {
        0
    }

    fn cpu_outp(&mut self, _port: rz80::RegT, _val: rz80::RegT) {}

    fn irq(&self, _ctrl_id: usize, _vec: u8) {}

    fn irq_cpu(&self) {}

    fn irq_ack(&self) -> rz80::RegT {
        0
    }

    fn irq_reti(&self) {

    }
}
