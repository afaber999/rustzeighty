use std::net::TcpListener;
//use std::net::TcpStream;
use std::io::{Read, Write};


use std::sync::Arc;
use std::sync::mpsc;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;use std::{thread};
use std::sync::mpsc::TryRecvError;

use crate::peripherals::iochannel::IOChannel;


pub struct TelnetChannel {
    rx_in : Receiver<u8>, 
    tx_out : Sender<u8>,
}

impl TelnetChannel {

    #[allow(dead_code)]
    pub fn new() -> Self {

        let listener = TcpListener::bind("127.0.0.1:23").expect("bind error");
		let (stream, _) = listener.accept().unwrap();
        let stream = Arc::new(stream);

        let rx_stream = stream.clone();
        let tx_stream = stream.clone();

        let (tx_in, rx_in) = mpsc::channel::<u8>();

        thread::spawn(move || loop {   
			loop {
                let mut buf= [0u8];
                let bytes = rx_stream.as_ref().read(&mut buf).unwrap();
                if bytes > 0 {
                    //println!("GOT CHAR bytes {} char {} {} ",bytes,  buf[0],   buf[0] as char);
                    tx_in.send( buf[0] ).unwrap();
                } else {
                }
  //              if buf[0] == 3 {
    //                panic!("CTRL-C break");
      //          }
//                println!("Got telnet char: {}", buf[0]);
            }
        });

        let (tx_out, rx_out) = mpsc::channel::<u8>();
        thread::spawn(move || loop {
    
            match rx_out.try_recv() {
                Ok(ch) => {
                    //print!("{}", ch as char);
                    let buf  = [ch];
                    tx_stream.as_ref().write(&buf).unwrap();
                },
                Err(TryRecvError::Empty) =>{} ,
                Err(TryRecvError::Disconnected) => panic!("Telnet channel disconnected"),
            }
        });
    
        Self {
            rx_in,
            tx_out
        }
    }
    
    #[allow(dead_code)]
    pub fn process_input(&mut self, channel : &mut dyn IOChannel) {
        loop {
            match self.rx_in.try_recv() {
                Ok(key) => {
                    if key == 3 {
                        break;
                    }
                    //println!("ADD KEY {}", key);
                    channel.add_in_char(key);
                },
                Err(TryRecvError::Empty) =>{ return } ,
                Err(TryRecvError::Disconnected) => panic!("Telnet channel disconnected"),
            } 
        }
    }

    #[allow(dead_code)]
    pub fn process_output(&mut self, channel : &mut dyn IOChannel) {
        while let Some(c) = channel.get_out_char() {
            self.tx_out.send(c).unwrap();
        }
    }
}
