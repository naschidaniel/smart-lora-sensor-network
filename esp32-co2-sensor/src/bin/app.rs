#![no_std]
#![no_main]

use core::fmt::Write;
use ee895::EE895;
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, ascii::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Delay, Rtc, Uart,
};
use esp_backtrace as _;
use esp_println::println;
use heapless::String;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    // init Watchdog and RTC
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.rwdt.disable();
    wdt.start(30u64.secs());

    // init delay
    let mut delay = Delay::new(&clocks);

    // init io
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // init LORA UART
    let config = Config {
        baudrate: 9600,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };
    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio17.into_push_pull_output(),
        io.pins.gpio16.into_floating_input(),
    );
    let mut serial1 = Uart::new_with_config(
        peripherals.UART1,
        Some(config),
        Some(pins),
        &clocks,
        &mut system.peripheral_clock_control,
    );

    // init I2C
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        10u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    let bus = shared_bus::BusManagerSimple::new(i2c);

    let mut sensor = EE895::new(bus.acquire_i2c()).unwrap();
    let mut warning: &str;
    let mut co2: f32;
    let mut temperature: f32;
    let mut pressure: f32;
    let mut co2_msg: String<20> = String::new();
    let mut pressure_msg: String<20> = String::new();
    let mut temperature_msg: String<20> = String::new();
    let mut lora_msg: String<100> = String::new();

    // init Embedded Graphics
    let text_style: MonoTextStyle<BinaryColor> = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
    let text_style_big = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);

    // onboard LED
    let mut led = io.pins.gpio2.into_push_pull_output();

    // init Display
    let interface = I2CDisplayInterface::new(bus.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    display.clear();

    loop {
        wdt.feed();
        led.set_high().unwrap();

        co2 = sensor.read_co2().unwrap();
        temperature = sensor.read_temperature().unwrap();
        pressure = sensor.read_pressure().unwrap();

        warning = match co2 {
            v if v <= 450.0 => "Fresh",
            v if v <= 700.0 => "Good",
            v if v <= 1000.0 => "Moderate",
            v if v <= 1500.0 => "Unhealthy",
            v if v <= 2500.0 => "Dangerous",
            _ => "Hazardous",
        };

        display.clear();

        co2_msg.clear();
        temperature_msg.clear();
        pressure_msg.clear();
        lora_msg.clear();
        write!(co2_msg, "CO2: {co2} ppm").unwrap();
        write!(temperature_msg, "T: {temperature} Deg C").unwrap();
        write!(pressure_msg, "P: {pressure} hPa").unwrap();
        write!(
            lora_msg,
            "SENSOR_CO2\t{co2_msg}\t{warning}\t{temperature_msg}\t{pressure_msg}\tCOMPLETE_CO2\n"
        )
        .unwrap();

        println!("{}", lora_msg);

        Text::with_alignment(
            co2_msg.as_str(),
            display.bounding_box().center() + Point::new(0, -20),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            warning,
            display.bounding_box().center() + Point::new(0, -2),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            temperature_msg.as_str(),
            display.bounding_box().center() + Point::new(0, 16),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            pressure_msg.as_str(),
            display.bounding_box().center() + Point::new(0, 32),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display and clear display buffer
        display.flush().unwrap();
        display.clear();

        serial1.write_str(lora_msg.as_str()).unwrap();

        // Wait 5 seconds
        led.set_low().unwrap();
        delay.delay_ms(5000u32);
    }
}
