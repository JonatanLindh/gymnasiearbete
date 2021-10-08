#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::{pac::tc2, prelude::*};
use panic_halt as _;

// #[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    pins.d3.into_output();
    pins.d11.into_output();

    let tc1 = dp.TC1;
    let tc2 = dp.TC2;

    tc1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tc1.tccr1b.write(|w| w.wgm1().bits(0b00).cs1().bits(0b01));
    tc1.timsk1.write(|w| w.toie1().bit(true));

    loop {
        // for duty in 0u8..255u8 {
        //     tc1.ocr1a.write(|w| unsafe { w.bits(duty as u16) });
        //     arduino_hal::delay_ms(20);
        // }
    }
}
