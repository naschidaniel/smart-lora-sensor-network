#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp_pico::entry;

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, ascii::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use heapless::String;
use rp2040_hal as hal;
use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
};
use ssd1306::{prelude::*, Ssd1306};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = rp_pico::hal::Watchdog::new(pac.WATCHDOG);
    watchdog.start(8.secs());

    let sio = Sio::new(pac.SIO);

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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    // init I2C
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        10u32.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let interface = ssd1306::I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // init Embedded Graphics
    let text_style: MonoTextStyle<BinaryColor> = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
    let text_style_big = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);

    let mut led_pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio25,
        rp_pico::hal::gpio::Output<rp_pico::hal::gpio::PushPull>,
    > = pins.led.into_push_pull_output();

    let uart_pins = (
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    let mut uart = rp_pico::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut moisture_msg: String<20> = String::new();
    write!(moisture_msg, "SOIL: 123.0 g/m^3").unwrap();
    let mut lora_msg: String<100> = String::new();

    loop {
        info!("on!");
        watchdog.feed();
        lora_msg.clear();
        write!(
            lora_msg,
            "SENSOR_MOISTURE\t{moisture_msg}\tCOMPLETE_MOISTURE\n"
        )
        .unwrap();
        led_pin.set_high().unwrap();

        Text::with_alignment(
            &moisture_msg.as_str(),
            display.bounding_box().center() + Point::new(0, 32),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display and clear display buffer
        display.flush().unwrap();
        display.clear();

        delay.delay_ms(1000);
        uart.write_str(lora_msg.as_str()).unwrap();
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(6000);
    }
}
