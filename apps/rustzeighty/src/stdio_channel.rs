
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::sync::mpsc::TryRecvError;
use std::{thread};

use std::io::{Write};
use std::io::stdout;
use crate::peripherals::iochannel::IOChannel;

pub struct StdioChannel {
    rx_in : Receiver<u8>, 
    tx_out : Sender<u8>,
}

impl StdioChannel {

    pub fn new() -> Self {
        let getch = getch::Getch::new();
        
        let (tx_in, rx_in) = mpsc::channel::<u8>();
        thread::spawn(move || loop {
            let ch = getch.getch().expect("read char error from stdin");
            if ch == 4 {
                panic!("CTRL-C break");
            }
            //println!(" E:{}", ch);
             tx_in.send( ch ).unwrap();

        });
    
        let (tx_out, rx_out) = mpsc::channel::<u8>();
        thread::spawn(move || loop {
    
            match rx_out.try_recv() {
                Ok(ch) => {
                    // if ch as char ==  '\\' {
                    //     panic!("break! on backslash");
                    // }
                    print!("{}", ch as char);
                    drop( stdout().flush());
                },
                Err(TryRecvError::Empty) =>{} ,
                Err(TryRecvError::Disconnected) => panic!("Stdio channel disconnected"),
            }
        });
    
        Self {
            rx_in,
            tx_out
        }
    }
    
    pub fn process_input(&mut self, channel : &mut dyn IOChannel) {
        loop {
            match self.rx_in.try_recv() {
                Ok(key) => {
                    if key == 4 {
                        break;
                    }
                    channel.add_in_char(key);
                },
                Err(TryRecvError::Empty) =>{ return } ,
                Err(TryRecvError::Disconnected) => panic!("Stdio channel disconnected"),
            } 
        }
    }

    pub fn process_output(&mut self, channel : &mut dyn IOChannel) {
        while let Some(c) = channel.get_out_char() {
            //println!(" A:{}", c);
            self.tx_out.send(c).unwrap();
        }
    }
}

