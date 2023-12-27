//! An async-await version of the Max31855 driver
//!
//! Intended for use with [embedded-hal-async].

use crate::{io_less, Error, FullResult, FullResultRaw, Unit};
use embedded_hal_async::spi::SpiDevice;

/// Trait enabling using the MAX31855
pub trait Max31855<Spi: SpiDevice> {
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    async fn read_thermocouple_raw(&mut self) -> Result<i16, Error<Spi>>;
    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    async fn read_thermocouple(&mut self, unit: Unit) -> Result<f32, Error<Spi>>;
    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    async fn read_all_raw(&mut self) -> Result<FullResultRaw, Error<Spi>>;
    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    async fn read_all(&mut self, unit: Unit) -> Result<FullResult, Error<Spi>>;
}

impl<SPI> Max31855<SPI> for SPI
where
    SPI: SpiDevice<u8>,
{
    /// Reads the thermocouple temperature and leave it as a raw ADC count. Checks if there is a fault but doesn't detect what kind of fault it is
    async fn read_thermocouple_raw(&mut self) -> Result<i16, Error<SPI>> {
        let mut buffer = [0; 2];
        self.transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiError)?;

        Ok(io_less::read_thermocouple_raw(buffer)?)
    }

    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    async fn read_thermocouple(&mut self, unit: Unit) -> Result<f32, Error<SPI>> {
        let raw = self.read_thermocouple_raw().await?;
        Ok(io_less::read_thermocouple(raw, unit))
    }

    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    async fn read_all_raw(&mut self) -> Result<FullResultRaw, Error<SPI>> {
        let mut buffer = [0; 4];
        self.transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SpiError)?;
        Ok(io_less::read_all_raw(buffer)?)
    }

    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    async fn read_all(&mut self, unit: Unit) -> Result<FullResult, Error<SPI>> {
        let res = self.read_all_raw().await?;
        Ok(io_less::read_all(res, unit))
    }
}
