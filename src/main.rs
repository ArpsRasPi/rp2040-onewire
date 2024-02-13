//! Implementation of the one-wire protocol for RP2040 using PIO
//!
//! Initially used to drive a DS18B20 digital thermometer
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionPio0, Pin},
        pac,
        pio::{PIOExt, PinDir},
        sio::Sio,
        watchdog::Watchdog,
    },
};

use pio_proc::pio_file;

#[entry]
fn main() -> ! {
    info!("Program start");

    // Device set up starts
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Device setup ends

    // Change the GPIO pin here for the data bus
    let pin_bus: Pin<_, FunctionPio0, _> = pins.gpio28.into_function();

    let program_with_defines = pio_file!(
        "./src/one_wire.pio",
        select_program("one_wire"),
        options(max_program_size = 32)
    );

    loop {}
}
// End of file
