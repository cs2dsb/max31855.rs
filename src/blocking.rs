//! A blocking (non-async) implementation of the Max31855 driver
//!
//! Intended for use with [embedded-hal].

use embedded_hal::{
    digital::{OutputPin, PinState},
    spi::{self, SpiDevice},
};

use crate::{
    Error, FullResult, FullResultRaw, Unit, io_less,
};

fn transfer<CS, SPI>(
    spi: &mut SPI,
    chip_select: &mut CS,
    buffer: &mut [u8],
) -> Result<(), Error<SPI, CS>>
where
    CS: OutputPin,
    SPI: SpiDevice<u8> + spi::ErrorType,
{
    chip_select
        .set_state(PinState::Low)
        .map_err(|e| Error::ChipSelectError(e))?;

    spi.transfer_in_place(buffer)
        .map_err(|e| Error::SpiError(e))?;

    chip_select
        .set_state(PinState::High)
        .map_err(|e| Error::ChipSelectError(e))
}

/// Trait enabling using the MAX31855
pub trait Max31855<Spi: SpiDevice, CS: OutputPin> {
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple_raw(&mut self, chip_select: &mut CS) -> Result<i16, Error<Spi, CS>>;
    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple(
        &mut self,
        chip_select: &mut CS,
        unit: Unit,
    ) -> Result<f32, Error<Spi, CS>>;
    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all_raw(&mut self, chip_select: &mut CS) -> Result<FullResultRaw, Error<Spi, CS>>;
    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all(&mut self, chip_select: &mut CS, unit: Unit) -> Result<FullResult, Error<Spi, CS>>;
}

impl<CS, SPI> Max31855<SPI, CS> for SPI
where
    CS: OutputPin,
    SPI: SpiDevice<u8>,
{
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple_raw(&mut self, chip_select: &mut CS) -> Result<i16, Error<SPI, CS>> {
        let mut buffer = [0; 2];
        transfer(self, chip_select, &mut buffer)?;

        Ok(io_less::read_thermocouple_raw(buffer)?)
    }

    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple(
        &mut self,
        chip_select: &mut CS,
        unit: Unit,
    ) -> Result<f32, Error<SPI, CS>> {
        let raw = self.read_thermocouple_raw(chip_select)?;
        Ok(io_less::read_thermocouple(raw, unit))
    }

    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all_raw(&mut self, chip_select: &mut CS) -> Result<FullResultRaw, Error<SPI, CS>> {
        let mut buffer = [0; 4];
        transfer(self, chip_select, &mut buffer)?;
        Ok(io_less::read_all_raw(buffer)?)
    }

    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all(&mut self, chip_select: &mut CS, unit: Unit) -> Result<FullResult, Error<SPI, CS>> {
        let res = self.read_all_raw(chip_select)?;
        Ok(io_less::read_all(res, unit))
    }
}
