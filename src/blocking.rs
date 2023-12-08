use bit_field::BitField;
use embedded_hal::{
    digital::{OutputPin, PinState},
    spi::{self, SpiDevice},
};

use crate::{
    bits_to_i16, Error, FullResult, FullResultRaw, Reading, Unit, FAULT_BIT,
    FAULT_GROUND_SHORT_BIT, FAULT_NO_THERMOCOUPLE_BIT, FAULT_VCC_SHORT_BIT, INTERNAL_BITS,
    THERMOCOUPLE_BITS,
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

        if buffer[1].get_bit(FAULT_BIT) {
            Err(Error::Fault)?
        }

        let raw = (buffer[0] as u16) << 8 | (buffer[1] as u16);

        let thermocouple = bits_to_i16(raw.get_bits(THERMOCOUPLE_BITS), 14, 4, 2);

        Ok(thermocouple)
    }

    /// Reads the thermocouple temperature and converts it into degrees in the provided unit. Checks if there is a fault but doesn't detect what kind of fault it is
    fn read_thermocouple(
        &mut self,
        chip_select: &mut CS,
        unit: Unit,
    ) -> Result<f32, Error<SPI, CS>> {
        self.read_thermocouple_raw(chip_select)
            .map(|r| unit.convert(Reading::Thermocouple.convert(r)))
    }

    /// Reads both the thermocouple and the internal temperatures, leaving them as raw ADC counts and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all_raw(&mut self, chip_select: &mut CS) -> Result<FullResultRaw, Error<SPI, CS>> {
        let mut buffer = [0; 4];
        transfer(self, chip_select, &mut buffer)?;

        let fault = buffer[1].get_bit(0);

        if fault {
            let raw = (buffer[2] as u16) << 8 | (buffer[3] as u16);

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

        let first_u16 = (buffer[0] as u16) << 8 | (buffer[1] as u16);
        let second_u16 = (buffer[2] as u16) << 8 | (buffer[3] as u16);

        let thermocouple = bits_to_i16(first_u16.get_bits(THERMOCOUPLE_BITS), 14, 4, 2);
        let internal = bits_to_i16(second_u16.get_bits(INTERNAL_BITS), 12, 16, 4);

        Ok(FullResultRaw {
            thermocouple,
            internal,
        })
    }

    /// Reads both the thermocouple and the internal temperatures, converts them into degrees in the provided unit and resolves faults to one of vcc short, ground short or missing thermocouple
    fn read_all(&mut self, chip_select: &mut CS, unit: Unit) -> Result<FullResult, Error<SPI, CS>> {
        self.read_all_raw(chip_select).map(|r| r.convert(unit))
    }
}
