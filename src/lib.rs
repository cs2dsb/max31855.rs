//! # max31855
//! 
//! Driver for [MAX31855 thermocouple converter](https://www.maximintegrated.com/en/products/sensors/MAX31855.html) using traits from `embedded-hal`.
//!
//! ## Features
//! * Read thermocouple temperature
//! * Read internal reference junction temperature
//! * Read fault data (missing thermocouple, short to ground or short to vcc)
//! * Supports 16-bit (thermocouple + fault only) or 32-bit (thermocouple, internal and full fault details)
//! * Supports Celsius, Fahrenheit or Kelvin units
//! * Supports returning raw (ADC count) readings
//!
//! ## Example:
//! ```
//!     let freq: Hertz = 4.mhz().into();
//!     let mode = Mode {
//!         polarity: Polarity::IdleLow,
//!         phase: Phase::CaptureOnFirstTransition
//!     };
//! 
//!     let mut spi = Spi::spi2(
//!         device.SPI2,
//!         (sck_pin, miso_pin, mosi_pin)
//!         mode,
//!         freq,
//!         clocks,
//!         &mut rcc.apb1
//!     );
//! 
//!     // Full 32-bit read, result contains both thermocouple and internal temperatures
//!     match spi.read_all(&mut cs_pin, Unit::Celsius) {
//!         Ok(v) => info!("Ok: {:?}", v),
//!         Err(e) => info!("Err: {:?}", e),
//!     }
//! 
//!     // Just thermocouple 16-bit read
//!     match spi.read_thermocouple(&mut cs_pin, Unit::Celsius) {
//!         Ok(v) => info!("Ok: {:?}", v),
//!         Err(e) => info!("Err: {:?}", e),
//!     }
//! ```

#![no_std]
#![deny(warnings, missing_docs)]

use embedded_hal::{
    blocking::spi::Transfer,
    digital::v2::OutputPin,
};
use bit_field::BitField;
use core::ops::RangeInclusive;

/// The bits that represent the thermocouple value when reading the first u16 from the sensor
const THERMOCOUPLE_BITS: RangeInclusive<usize> = 2..=15;
/// The bit that indicates some kind of fault when reading the first u16 from the sensor
const FAULT_BIT: usize = 0;
/// The bits that represent the internal value when reading the second u16 from the sensor
const INTERNAL_BITS: RangeInclusive<usize> = 4..=15;
/// The bit that indicates a short-to-vcc fault when reading the second u16 from the sensor
const FAULT_VCC_SHORT_BIT: usize = 2;
/// The bit that indicates a short-to-gnd fault when reading the second u16 from the sensor
const FAULT_GROUND_SHORT_BIT: usize = 1;
/// The bit that indicates a missing thermocouple fault when reading the second u16 from the sensor
const FAULT_NO_THERMOCOUPLE_BIT: usize = 0;

/// Possible errors returned by this crate
#[derive(Debug)]
pub enum Error<SpiE, CsE> {
    /// An error returned by a call to Transfer::transfer
    SpiError(SpiE),
    /// An error returned by a call to OutputPin::{set_high, set_low}
    ChipSelectError(CsE),
    /// The fault bit (16) was set in the response from the MAX31855
    Fault,
    /// The SCV fault bit (2) was set in the response from the MAX31855
    VccShortFault,
    /// The SCG fault bit (1) was set in the response from the MAX31855
    GroundShortFault,
    /// The OC fault bit (0) was set in the response from the MAX31855
    MissingThermocoupleFault,
}

/// The temperature unit to use
#[derive(Clone, Copy, Debug)]
pub enum Unit {
    /// Degrees Celsius
    Celsius,
    /// Degrees Fahrenheit
    Fahrenheit,
    /// Degrees Kelvin
    Kelvin,
}

impl Unit {
    /// Converts degrees celsius into this unit
    pub fn convert(&self, celsius: f32) -> f32 {
        match self {
            Unit::Celsius => celsius,
            Unit::Fahrenheit => celsius * 1.8 + 32.,
            Unit::Kelvin => celsius + 273.15,
        }
    }
}

/// Possible MAX31855 readings
pub enum Reading {
    /// The attached thermocouple
    Thermocouple,
    /// The internal reference junction
    Internal,
}

impl Reading {
    /// Convert the raw ADC count into degrees celsius
    pub fn convert(self, count: i16) -> f32 {
        let count = count as f32;
        match self {
            Reading::Thermocouple => count * 0.25,
            Reading::Internal => count * 0.0625,
        }
    }
}

enum CsState {
    High,
    Low,
}
use CsState::*;

fn set_cs<CS, SpiE, CsE>(cs: &mut CS, state: CsState) -> Result<(), Error<SpiE, CsE>>
where
    CS: OutputPin<Error = CsE>, 
{
    let result = match state {
        CsState::High => cs.set_high(),
        CsState::Low => cs.set_low(),
    };

    result.map_err(|e| Error::ChipSelectError(e))
}

fn transfer<CS, SPI, SpiE, CsE>(spi: &mut SPI, chip_select: &mut CS, buffer: &mut [u8]) -> Result<(), Error<SpiE, CsE>>
where
    CS: OutputPin<Error = CsE>, 
    SPI: Transfer<u8, Error = SpiE>,
{
    set_cs(chip_select, Low)?;

    spi.transfer(buffer).map_err(|e| Error::SpiError(e))?;

    set_cs(chip_select, High)
}

fn bits_to_i16(bits: u16, len: usize, divisor: i16, shift: usize) -> i16 {
    let negative = bits.get_bit(len - 1);
    if negative {
        (bits << shift) as i16 / divisor
    } else {
        bits as i16
    }
}

/// Represents the data contained in a full 32-bit read from the MAX31855 as raw ADC counts
#[derive(Debug)]
pub struct FullResultRaw {
    /// The temperature of the thermocouple as raw ADC counts
    pub thermocouple: i16,
    /// The temperature of the MAX31855 reference junction as raw ADC counts
    pub internal: i16,
}

impl FullResultRaw {
    /// Convert the raw ADC counts into degrees in the provided Unit
    pub fn convert(self, unit: Unit) -> FullResult {        
        let thermocouple = unit.convert(Reading::Thermocouple.convert(self.thermocouple));        
        let internal = unit.convert(Reading::Internal.convert(self.internal));

        FullResult {
            thermocouple,
            internal,
            unit,
        }
    }
}

/// Represents the data contained in a full 32-bit read from the MAX31855 as degrees in the included Unit
#[derive(Debug)]
pub struct FullResult {
    /// The temperature of the thermocouple
    pub thermocouple: f32,
    /// The temperature of the MAX31855 reference junction
    pub internal: f32,
    /// The unit that the temperatures are in
    pub unit: Unit,
}

/// Trait enabling using the MAX31855
pub trait Max31855<SpiE, CsE, CS> {
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple_raw(&mut self, chip_select: &mut CS) -> Result<i16, Error<SpiE, CsE>>;
    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple(&mut self, chip_select: &mut CS, unit: Unit) -> Result<f32, Error<SpiE, CsE>>;
    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all_raw(&mut self, chip_select: &mut CS) -> Result<FullResultRaw, Error<SpiE, CsE>>;
    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all(&mut self, chip_select: &mut CS, unit: Unit) -> Result<FullResult, Error<SpiE, CsE>>;
}

impl<CS, SPI, SpiE, CsE> Max31855<SpiE, CsE, CS> for SPI
where
    CS: OutputPin<Error = CsE>, 
    SPI: Transfer<u8, Error = SpiE>,
{
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple_raw(&mut self, chip_select: &mut CS) -> Result<i16, Error<SpiE, CsE>> {
        let mut buffer = [0; 2];
        transfer(self, chip_select, &mut buffer)?;

        if buffer[1].get_bit(FAULT_BIT) {
            Err(Error::Fault)?
        }

        let raw = (buffer[0] as u16) << 8 |
                  (buffer[1] as u16) << 0;

        let thermocouple = bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2);
        
        Ok(thermocouple)
    }

    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple(&mut self, chip_select: &mut CS, unit: Unit) -> Result<f32, Error<SpiE, CsE>> {
        self
            .read_thermocouple_raw(chip_select)
            .map(|r| unit.convert(Reading::Thermocouple.convert(r)))
    }

    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all_raw(&mut self, chip_select: &mut CS) -> Result<FullResultRaw, Error<SpiE, CsE>> {
        let mut buffer = [0; 4];
        transfer(self, chip_select, &mut buffer)?;

        let fault = buffer[1].get_bit(0);

        if fault {
            let raw = (buffer[2] as u16) << 8 |
                      (buffer[3] as u16) << 0;

            if raw.get_bit(FAULT_NO_THERMOCOUPLE_BIT) {
                Err(Error::MissingThermocoupleFault)?
            } else if raw.get_bit(FAULT_GROUND_SHORT_BIT) {
                Err(Error::GroundShortFault)?
            } else if raw.get_bit(FAULT_VCC_SHORT_BIT) {
                Err(Error::VccShortFault)?
            } else {
                // This should impossible, one of the other fields should be set as well
                // but handled here just-in-case
                Err(Error::Fault)?
            }
        }

        let first_u16 = (buffer[0] as u16) << 8 |
                        (buffer[1] as u16) << 0;
        let second_u16 = (buffer[2] as u16) << 8 |
                         (buffer[3] as u16) << 0;
        
        let thermocouple = bits_to_i16(first_u16.get_bits(THERMOCOUPLE_BITS), 14, 4, 2);
        let internal = bits_to_i16(second_u16.get_bits(INTERNAL_BITS), 12, 16, 4);

        Ok(FullResultRaw {
            thermocouple,
            internal,
        })
    }

    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all(&mut self, chip_select: &mut CS, unit: Unit) -> Result<FullResult, Error<SpiE, CsE>> {
        self
            .read_all_raw(chip_select)
            .map(|r| r.convert(unit))
    }
}