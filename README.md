# max31855

Driver for [MAX31855 thermocouple converter](https://www.maximintegrated.com/en/products/sensors/MAX31855.html) using traits from `embedded-hal`.

[![Crate](https://img.shields.io/crates/v/max31855.svg)](https://crates.io/crates/max31855)
[![Documentation](https://docs.rs/max31855/badge.svg)](https://docs.rs/max31855)

## Features
* Read thermocouple temperature
* Read internal reference junction temperature
* Read fault data (missing thermocouple, short to ground or short to vcc)
* Supports 16-bit (thermocouple + fault only) or 32-bit (thermocouple, internal and full fault details)
* Supports Celsius, Fahrenheit or Kelvin units
* Supports returning raw (ADC count) readings

## Example:
```
    let freq: Hertz = 4.mhz().into();
    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };

    let mut spi = Spi::spi2(
        device.SPI2,
        (sck_pin, miso_pin, mosi_pin)
        mode,
        freq,
        clocks,
        &mut rcc.apb1
    );

    // Full 32-bit read, result contains both thermocouple and internal temperatures
    match spi.read_all(&mut cs_pin, Unit::Celsius) {
        Ok(v) => info!("Ok: {:?}", v),
        Err(e) => info!("Err: {:?}", e),
    }

    // Just thermocouple 16-bit read
    match spi.read_thermocouple(&mut cs_pin, Unit::Celsius) {
        Ok(v) => info!("Ok: {:?}", v),
        Err(e) => info!("Err: {:?}", e),
    }
```

Free and open source software distributed under the terms of both the [MIT License][lm] and the [Apache License 2.0][la].

[lm]: LICENSE-MIT
[la]: LICENSE-APACHE