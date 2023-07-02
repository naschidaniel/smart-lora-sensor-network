#![no_std]
#![no_main]

use embedded_graphics::{
    image::{Image, ImageRaw, ImageRawLE},
    mono_font::{ascii::FONT_8X13, ascii::FONT_9X15, ascii::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
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
    let text_style_small = MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE);
    let text_style_medium = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);
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

    let del_var = 30_u32.secs();
    let mut lora_msg: String<150> = String::new();
    let mut co2_msg: String<50> = String::from("-");
    let mut co2_warning_msg: String<50> = String::from("-");
    let mut t_msg: String<50> = String::from("-");
    let mut p_msg: String<50> = String::from("-");
    let mut soil1_msg: String<50> = String::from("-");
    let mut soil2_msg: String<50> = String::from("-");
    let mut soil3_msg: String<50> = String::from("-");
    let mut co2_sensor_available = false;
    let mut display_values_update = false;

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
                    let _ = lora_msg.push(c);
                }
                Err(_) => {
                    println!("Error reading");
                    break;
                }
            }
        }
        println!("{}", lora_msg);

        if lora_msg.contains("SENSOR_MOISTURE") && lora_msg.contains("COMPLETE_MOISTURE") {
            println!("{}", lora_msg);
            display_values_update = true;
            let moisture_sensor_msg: Vec<&str, 8> = lora_msg.split('\t').collect();
            soil1_msg.clear();
            soil1_msg = moisture_sensor_msg[2].into();
            soil2_msg.clear();
            soil2_msg = moisture_sensor_msg[4].into();
            soil3_msg.clear();
            soil3_msg = moisture_sensor_msg[6].into();
        }

        if lora_msg.contains("SENSOR_CO2") && lora_msg.contains("COMPLETE_CO2") {
            println!("{}", lora_msg);
            co2_sensor_available = true;
            display_values_update = true;

            let co2_sensor_msg: Vec<&str, 6> = lora_msg.split('\t').collect();
            co2_msg.clear();
            co2_msg = co2_sensor_msg[1].into();
            co2_warning_msg.clear();
            co2_warning_msg = co2_sensor_msg[2].into();
            t_msg.clear();
            t_msg = co2_sensor_msg[3].into();
            p_msg.clear();
            p_msg = co2_sensor_msg[4].into();

            timer0.start(del_var);
        }

        if timer0.wait() == Ok(()) {
            println!("reset timer");
            co2_sensor_available = false;
        }

        if co2_sensor_available && display_values_update {
            display_values_update = false;
            display.clear(Rgb565::BLACK).unwrap();
            color = match &co2_warning_msg {
                v if v == "Fresh" => Rgb565::BLUE,
                v if v == "Good" => Rgb565::GREEN,
                v if v == "Moderate" => Rgb565::CSS_ORANGE,
                v if v == "Unhealthy" => Rgb565::CSS_INDIAN_RED,
                v if v == "Dangerous" => Rgb565::CSS_VIOLET,
                _ => Rgb565::CSS_DARK_VIOLET,
            };
            Text::new(&co2_msg, Point::new(05, 20), text_style_big)
                .draw(&mut display)
                .unwrap();
            Text::new(&co2_warning_msg, Point::new(05, 40), text_style_big)
                .draw(&mut display)
                .unwrap();
            Circle::new(Point::new(127, 03), 30)
                .into_styled(PrimitiveStyle::with_fill(color))
                .draw(&mut display)
                .unwrap();
            Text::new(&t_msg, Point::new(05, 60), text_style_medium)
                .draw(&mut display)
                .unwrap();
            Text::new(&p_msg, Point::new(05, 75), text_style_medium)
                .draw(&mut display)
                .unwrap();
            Text::new(&soil1_msg, Point::new(05, 90), text_style_small)
                .draw(&mut display)
                .unwrap();
            Text::new(&soil2_msg, Point::new(05, 105), text_style_small)
                .draw(&mut display)
                .unwrap();
            Text::new(&soil3_msg, Point::new(05, 120), text_style_small)
                .draw(&mut display)
                .unwrap();
        }
        if !co2_sensor_available {
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
