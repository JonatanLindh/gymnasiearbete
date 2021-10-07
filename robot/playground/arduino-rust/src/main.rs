#![no_std]
#![no_main]

use arduino_hal::prelude::*;
use panic_halt as _;

// #[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    pins.d9.into_output();
    let mut led = pins.d13.into_output_high();

    let tc1 = dp.TC1;
    tc1.tccr1a
        .write(|w| w.wgm1().bits(0b01).com1a().match_clear());
    tc1.tccr1b
        .write(|w| w.wgm1().bits(0b01).cs1().prescale_64());

    loop {
        for duty in 0u8..255u8 {
            tc1.ocr1a.write(|w| unsafe { w.bits(duty as u16) });
            arduino_hal::delay_ms(20);
        }
    }
}
