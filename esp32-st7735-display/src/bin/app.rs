#![no_std]
#![no_main]

use embedded_graphics::image::{Image, ImageRaw, ImageRawLE};
use embedded_graphics::{
    mono_font::{ascii::FONT_9X15, ascii::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
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
use heapless::Vec;
use st7735_lcd;
use st7735_lcd::Orientation;

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
    let mut timer0 = timer_group0.timer0;

    // init Watchdog and RTC
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.rwdt.disable();
    wdt.start(10u64.secs());

    // delay
    let mut delay = Delay::new(&clocks);

    // Embedded Graphics
    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);
    let text_style_big = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::WHITE);

    // init io
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut color: Rgb565;
    println!("Hello World!");

    // onboard LED
    let mut led = io.pins.gpio2.into_push_pull_output();

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

    // SPI Display Settings
    let sck = io.pins.gpio18; // sck
    let sda = io.pins.gpio23; // sda
    let miso = io.pins.gpio19.into_push_pull_output(); // A0
    let cs = io.pins.gpio5; // CS
    let dc = io.pins.gpio13.into_push_pull_output(); // dc not connected
    let rst = io.pins.gpio14.into_push_pull_output();

    let spi = Spi::new(
        peripherals.SPI2,
        sck,
        sda,
        dc,
        cs,
        60u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut display = st7735_lcd::ST7735::new(spi, miso, rst, true, false, 160, 128);

    display.init(&mut delay).unwrap();
    display.set_orientation(&Orientation::Landscape).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    display.set_offset(0, 0);

    let image_raw: ImageRawLE<Rgb565> =
        ImageRaw::new(include_bytes!("../../assets/ferris.raw"), 86);
    display.clear(Rgb565::BLACK).unwrap();
    let image: Image<_> = Image::new(&image_raw, Point::new(34, 30));
    image.draw(&mut display).unwrap();

    delay.delay_ms(5000u32);

    let del_var = 20_u32.secs();
    let mut lora_msg: String<100> = String::new();
    timer0.start(del_var);

    loop {
        wdt.feed();
        led.set_high().unwrap();

        serial1.flush().unwrap();
        lora_msg.clear();
        loop {
            let byte = serial1.read();
            match byte {
                Ok(b) => {
                    let c: char = char::from(b);
                    lora_msg.push(c);
                }
                Err(_) => {
                    println!("Error reading");
                    break;
                }
            }
        }
        if lora_msg.contains("SENSOR_MOISTURE") && lora_msg.contains("COMPLETE_MOISTURE") {
            println!("{}", lora_msg);
        }

        if lora_msg.contains("SENSOR_CO2") && lora_msg.contains("COMPLETE_CO2") {
            println!("{}", lora_msg);
            let lora_msg: Vec<&str, 6> = lora_msg.split('\t').collect();

            color = match lora_msg[2] {
                v if v == "Fresh" => Rgb565::BLUE,
                v if v == "Good" => Rgb565::GREEN,
                v if v == "Moderate" => Rgb565::CSS_ORANGE,
                v if v == "Unhealthy" => Rgb565::CSS_INDIAN_RED,
                v if v == "Dangerous" => Rgb565::CSS_VIOLET,
                _ => Rgb565::CSS_DARK_VIOLET,
            };
            display.clear(color).unwrap();

            Text::new(lora_msg[1], Point::new(05, 30), text_style_big)
                .draw(&mut display)
                .unwrap();
            Text::new(lora_msg[2], Point::new(05, 55), text_style_big)
                .draw(&mut display)
                .unwrap();
            Text::new(lora_msg[3], Point::new(05, 80), style)
                .draw(&mut display)
                .unwrap();
            Text::new(lora_msg[4], Point::new(05, 105), style)
                .draw(&mut display)
                .unwrap();
            timer0.start(del_var);
        }

        if timer0.wait() == Ok(()) {
            println!("reset timer");
            display.clear(Rgb565::RED).unwrap();

            Text::new(
                "The CO2-Sensor\nis not\navailable!",
                Point::new(05, 30),
                text_style_big,
            )
            .draw(&mut display)
            .unwrap();
        }
        led.set_low().unwrap();
        delay.delay_ms(500u32);
    }
}
