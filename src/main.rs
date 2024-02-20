//! Implementation of the one-wire protocol for RP2040 using PIO
//!
//! Initially used to drive a DS18B20 digital thermometer
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, PinState};
use panic_probe as _;

use pio::{Instruction, SetDestination};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionPio0, Pin},
        pac::{CorePeripherals, Peripherals},
        pio::{PIOBuilder, PIOExt, PinDir, ShiftDirection},
        sio::Sio,
        watchdog::Watchdog,
    }, Pins, };

use pio_proc::pio_file;

#[entry]
fn main() -> ! {
    info!("Program start");

    // Device set up starts
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Device setup ends

    // Get a handle to the LED for feedback
    let mut led_pin = pins
        .gpio16
        .into_push_pull_output_in_state(PinState::Low);
    // Get some pins for signalling different operations that can be used
    // to trigger logic analyser captures
    
    let mut dbg_reset_pin = pins
        .gpio10
        .into_push_pull_output_in_state(PinState::Low);
    let mut dbg_write_pin = pins
        .gpio11
        .into_push_pull_output_in_state(PinState::Low);
    let mut dbg_read_pin = pins
        .gpio12
        .into_push_pull_output_in_state(PinState::Low);
    
    // Change the GPIO pin here for the data bus
    info!("Configuring PIO");
    let pin_bus: Pin<_, FunctionPio0, _> = pins.gpio28.into_function();

    let program_with_defines = pio_file!(
        "./src/one_wire.pio",
        select_program("one_wire"),
        options(max_program_size = 32)
    );

    let program = program_with_defines.program;

    // Initialise and start the PIO
    let (mut pio, sm0, _sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (1200, 0); // I *think* this will set each clock cycle to be 10us!
    let (mut sm, mut rx, mut tx) = PIOBuilder::from_program(installed)
        .set_pins(pin_bus.id().num, 1)
        .in_pin_base(pin_bus.id().num)
        .clock_divisor_fixed_point(int, frac)
        .out_shift_direction(ShiftDirection::Right)
        .build(sm0);

    sm.set_pindirs([(pin_bus.id().num, PinDir::Output)]);
    sm.set_pins([(pin_bus.id().num, bsp::hal::pio::PinState::High)]);

    info!("Starting PIO");
    let mut running_sm = sm.start();

    loop {
        delay.delay_ms(1000);
        info!("Starting Loop");
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(50);

        // Do a reset and detect presence
        // Push 0 to tx - this is the reset/detect presence instruction
        // Strobe dbg pin
        dbg_reset_pin.set_high().unwrap();
        dbg_reset_pin.set_low().unwrap();
        running_sm.exec_instruction(Instruction {
            side_set: None,
            delay: 0,
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: 1,
            },
        });
        info!("Exec instruction sent to JMP to RESET/DETECT");
        // The reset should take 960+us, so lets delay a couple of ms.
        delay.delay_ms(2);
        // Pull response from rx FIFO
        if let Some(result) = rx.read() {
            // A 0 result means that the pin was low when read.
            // This is the indicator of at least one device present
            if result == 0 {
                info!("**** DEVICE DETECTED ****");
            } else {
                info!("No device detected");
            }
        } else {
            // This means that the rx fifo could not be pulled, probably because it was empty
            info!("Nothing returned from rx FIFO");
        }

        // Do a write of some bits
        let data :u8 = 0b01010101;
        dbg_write_pin.set_high().unwrap();
        dbg_write_pin.set_low().unwrap();
        
        // First, put 1 into x scratch register
        running_sm.exec_instruction(Instruction {
            side_set: None,
            delay: 0,
            operands: pio::InstructionOperands::SET { destination: SetDestination::X, data: 1 }
        });
        info!("Exec instruction sent for SET X 1");
        // Then jump to read/write
        running_sm.exec_instruction(Instruction {
            side_set: None,
            delay: 0,
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: 8,
            },
        });
        info!("Exec instruction sent to JMP to READ_WRITE");
        // That'll block for the bit count, so send it
        if tx.write(8) {
            info!("Sent bit count (8)");
            // Send data
            if tx.write(data as u32) {
                info!("Sent data.");
            }
            else {
                info!("Failed to send data.");
            }
        }
        else {
            info!("Failed to write bit count");
        }

        // Do a read of some data
        dbg_read_pin.set_high().unwrap();
        dbg_read_pin.set_low().unwrap();
        // First, put 0 into x scratch register
        running_sm.exec_instruction(Instruction {
            side_set: None,
            delay: 0,
            operands: pio::InstructionOperands::SET { destination: SetDestination::X, data: 0 }
        });
        info!("Exec instruction sent for SET X 0");
        // Then jump to read/write
        running_sm.exec_instruction(Instruction {
            side_set: None,
            delay: 0,
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: 8,
            },
        });
        info!("Exec instruction sent to JMP to READ_WRITE");
        // That'll block for the bit count, so send it
        if tx.write(3) {
            info!("Sent bit count (3)");
            // Read data
            // Wait for the read to complete.
            // Each read is about 100us so we'll wait for a couple of ms
            delay.delay_ms(2);
            if let Some(result) = rx.read() {
                info!("Did a read! Got result {}", result);
            }
            else {
                info!("Did not receive result.");
            }
        }
        else {
            info!("Failed to write read bit count");
        }
    }
}

// End of file
