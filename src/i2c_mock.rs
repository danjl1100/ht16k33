//! # i2c_mock
//!
//! A mock I2C library to support using the [HT16K33](../struct.HT16K33.html) driver on non-Linux systems that do
//! not have I2C support.

use core::fmt;

use crate::constants::ROWS_SIZE;
use crate::types::DisplayDataAddress;

/// Mock error to satisfy the I2C trait.
#[derive(Debug)]
pub struct I2cMockError;

#[cfg(feature = "std")]
impl std::error::Error for I2cMockError {}

impl fmt::Display for I2cMockError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "I2c MockError")
    }
}

/// The mock I2C state.
///
/// # Example
///
/// ```
/// use ht16k33::i2c_mock::I2cMock;
/// # fn main() {
///
/// // Create an I2cMock.
/// let i2c_mock = I2cMock::new();
///
/// # }
/// ```
pub struct I2cMock {
    /// Display RAM state.
    pub data_values: [u8; ROWS_SIZE],
}

impl I2cMock {
    /// Create an I2cMock.
    pub fn new() -> Self {
        I2cMock {
            data_values: [0; ROWS_SIZE],
        }
    }
}

mod blocking {
    use super::{I2cMock, I2cMockError};
    use embedded_hal as hal;

    impl hal::i2c::Error for I2cMockError {
        fn kind(&self) -> hal::i2c::ErrorKind {
            hal::i2c::ErrorKind::Other
        }
    }

    impl hal::i2c::ErrorType for I2cMock {
        type Error = I2cMockError;
    }
    impl hal::i2c::I2c for I2cMock {
        fn transaction(
            &mut self,
            address: u8,
            mut operations: &mut [hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            while let Some((first, rest)) = operations.split_first_mut() {
                operations = rest;
                match first {
                    hal::i2c::Operation::Read(_) => todo!(),
                    hal::i2c::Operation::Write(write_bytes) => {
                        if matches!(operations.first(), Some(hal::i2c::Operation::Read(_))) {
                            let Some((hal::i2c::Operation::Read(read_bytes), rest)) =
                                operations.split_first_mut()
                            else {
                                unreachable!()
                            };
                            operations = rest;
                            self.write_read(address, write_bytes, read_bytes)?;
                        } else {
                            self.write(address, write_bytes)?;
                        }
                    }
                }
            }
            Ok(())
        }
    }
}

mod non_blocking {
    use super::I2cMock;
    use embedded_hal_async as hal;

    impl hal::i2c::I2c for I2cMock {
        async fn transaction(
            &mut self,
            address: u8,
            mut operations: &mut [hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            while let Some((first, rest)) = operations.split_first_mut() {
                operations = rest;
                match first {
                    hal::i2c::Operation::Read(_) => todo!(),
                    hal::i2c::Operation::Write(write_bytes) => {
                        if matches!(operations.first(), Some(hal::i2c::Operation::Read(_))) {
                            let Some((hal::i2c::Operation::Read(read_bytes), rest)) =
                                operations.split_first_mut()
                            else {
                                unreachable!()
                            };
                            operations = rest;
                            self.write_read(address, write_bytes, read_bytes)?;
                        } else {
                            self.write(address, write_bytes)?;
                        }
                    }
                }
            }
            Ok(())
        }
    }
}

impl I2cMock {
    /// `write_read` implementation.
    ///
    /// # Arguments
    ///
    /// * `_address` - The slave address. Ignored.
    /// * `bytes` - The command/address instructions to be written.
    /// * `buffer` - The read results.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal::i2c::I2c;
    /// # use ht16k33::i2c_mock::I2cMock;
    /// # fn main() {
    /// let mut i2c_mock = I2cMock::new();
    ///
    /// let mut read_buffer = [0u8; 16];
    /// i2c_mock.write_read(0, &[ht16k33::DisplayDataAddress::ROW_0.bits()], &mut read_buffer);
    ///
    /// # }
    /// ```
    fn write_read(
        &mut self,
        _address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), I2cMockError> {
        // The `bytes` have the `data_address` command + index to start reading from,
        // need to clear the command to extract the starting index.
        let mut data_offset = (bytes[0] ^ DisplayDataAddress::ROW_0.bits()) as usize;

        for value in buffer.iter_mut() {
            *value = self.data_values[data_offset];

            // The HT16K33 supports auto-increment and wrap-around, emulate that.
            data_offset = (data_offset + 1) % self.data_values.len();
        }

        Ok(())
    }

    /// `write` implementation.
    ///
    /// # Arguments
    ///
    /// * `_address` - The slave address. Ignored.
    /// * `bytes` - The command/address instructions to be written.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal::i2c::I2c;
    /// # use ht16k33::i2c_mock::I2cMock;
    /// # fn main() {
    /// let mut i2c_mock = I2cMock::new();
    ///
    /// // First value is the data address, remaining values are to be written
    /// // starting at the data address which auto-increments and then wraps.
    /// let write_buffer = [ht16k33::DisplayDataAddress::ROW_0.bits(), 0u8, 0u8];
    ///
    /// i2c_mock.write(0, &write_buffer);
    ///
    /// # }
    /// ```
    fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), I2cMockError> {
        // "Command-only" writes are length 1 and write-only, and cannot be read back,
        // discard them for simplicity.
        if bytes.len() == 1 {
            return Ok(());
        }

        // Other writes have data, store them.
        let mut data_offset = (bytes[0] ^ DisplayDataAddress::ROW_0.bits()) as usize;
        let data = &bytes[1..];

        for value in data.iter() {
            self.data_values[data_offset] = *value;

            // The HT16K33 supports auto-increment and wrap-around, emulate that.
            data_offset = (data_offset + 1) % self.data_values.len();
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADDRESS: u8 = 0;

    #[test]
    fn new() {
        let _i2c_mock = I2cMock::new();
    }

    #[test]
    fn write() {
        let mut i2c_mock = I2cMock::new();

        let write_buffer = [super::DisplayDataAddress::ROW_0.bits(), 1u8, 1u8];
        i2c_mock.write(ADDRESS, &write_buffer).unwrap();

        for value in 0..i2c_mock.data_values.len() {
            match value {
                0 | 1 => assert_eq!(
                    i2c_mock.data_values[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
                _ => assert_eq!(
                    i2c_mock.data_values[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
            }
        }
    }

    #[test]
    fn write_with_offset() {
        let mut i2c_mock = I2cMock::new();

        let offset = 4u8;
        let write_buffer = [super::DisplayDataAddress::ROW_0.bits() | offset, 1u8, 1u8];
        i2c_mock.write(ADDRESS, &write_buffer).unwrap();

        for value in 0..i2c_mock.data_values.len() {
            match value {
                4 | 5 => assert_eq!(
                    i2c_mock.data_values[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
                _ => assert_eq!(
                    i2c_mock.data_values[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
            }
        }
    }

    #[test]
    fn write_with_wraparound() {
        let mut i2c_mock = I2cMock::new();

        // Match the data values size, +2 to wrap around, +1 for the data command.
        let mut write_buffer = [1u8; super::ROWS_SIZE + 3];
        write_buffer[0] = super::DisplayDataAddress::ROW_0.bits();

        // These values should wrap and end up at indexes 0 & 1.
        write_buffer[write_buffer.len() - 1] = 2;
        write_buffer[write_buffer.len() - 2] = 2;

        i2c_mock.write(ADDRESS, &write_buffer).unwrap();

        for value in 0..i2c_mock.data_values.len() {
            match value {
                0 | 1 => assert_eq!(
                    i2c_mock.data_values[value], 2,
                    "index [{}] should be 2, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
                _ => assert_eq!(
                    i2c_mock.data_values[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
            }
        }
    }

    #[test]
    fn write_with_wraparound_and_offset() {
        let mut i2c_mock = I2cMock::new();

        // Match the data values size, +2 to wrap around, +1 for the data command.
        let mut write_buffer = [1u8; super::ROWS_SIZE + 3];

        let offset = 4u8;
        write_buffer[0] = super::DisplayDataAddress::ROW_0.bits() | offset;

        // These values should wrap and end up at indexes 4 & 5.
        write_buffer[write_buffer.len() - 1] = 2;
        write_buffer[write_buffer.len() - 2] = 2;

        i2c_mock.write(ADDRESS, &write_buffer).unwrap();

        for value in 0..i2c_mock.data_values.len() {
            match value {
                4 | 5 => assert_eq!(
                    i2c_mock.data_values[value], 2,
                    "index [{}] should be 2, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
                _ => assert_eq!(
                    i2c_mock.data_values[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, i2c_mock.data_values[value]
                ),
            }
        }
    }

    #[test]
    fn write_read() {
        let mut i2c_mock = I2cMock::new();

        i2c_mock.data_values[0] = 1;
        i2c_mock.data_values[1] = 1;

        let mut read_buffer = [0u8; super::ROWS_SIZE];
        i2c_mock
            .write_read(
                ADDRESS,
                &[super::DisplayDataAddress::ROW_0.bits()],
                &mut read_buffer,
            )
            .unwrap();

        for value in 0..read_buffer.len() {
            match value {
                0 | 1 => assert_eq!(
                    read_buffer[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, read_buffer[value]
                ),
                _ => assert_eq!(
                    read_buffer[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, read_buffer[value]
                ),
            }
        }
    }

    #[test]
    fn write_read_offset() {
        let mut i2c_mock = I2cMock::new();

        i2c_mock.data_values[2] = 1;
        i2c_mock.data_values[3] = 1;

        let mut read_buffer = [0u8; 4];

        let offset = 2u8;
        i2c_mock
            .write_read(
                ADDRESS,
                &[super::DisplayDataAddress::ROW_0.bits() | offset],
                &mut read_buffer,
            )
            .unwrap();

        for value in 0..read_buffer.len() {
            match value {
                0 | 1 => assert_eq!(
                    read_buffer[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, read_buffer[value]
                ),
                _ => assert_eq!(
                    read_buffer[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, read_buffer[value]
                ),
            }
        }
    }

    #[test]
    fn write_read_wraparound() {
        let mut i2c_mock = I2cMock::new();

        i2c_mock.data_values[2] = 1;
        i2c_mock.data_values[3] = 1;

        let mut read_buffer = [0u8; super::ROWS_SIZE + 4];

        i2c_mock
            .write_read(
                ADDRESS,
                &[super::DisplayDataAddress::ROW_0.bits()],
                &mut read_buffer,
            )
            .unwrap();

        for value in 0..read_buffer.len() {
            match value {
                2 | 3 | 18 | 19 => assert_eq!(
                    read_buffer[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, read_buffer[value]
                ),
                _ => assert_eq!(
                    read_buffer[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, read_buffer[value]
                ),
            }
        }
    }

    #[test]
    fn write_read_wraparound_and_offset() {
        let mut i2c_mock = I2cMock::new();

        i2c_mock.data_values[0] = 1;
        i2c_mock.data_values[1] = 1;

        let mut read_buffer = [0u8; super::ROWS_SIZE];

        let offset = 4u8;
        i2c_mock
            .write_read(
                ADDRESS,
                &[super::DisplayDataAddress::ROW_0.bits() | offset],
                &mut read_buffer,
            )
            .unwrap();

        for value in 0..read_buffer.len() {
            match value {
                // The indexes will be 12/13 b/c the data values are at 1/2, but the read is offset
                // by 4, so the read buffer will wraparound to load those values.
                12 | 13 => assert_eq!(
                    read_buffer[value], 1,
                    "index [{}] should be 1, found [{}]",
                    value, read_buffer[value]
                ),
                _ => assert_eq!(
                    read_buffer[value], 0,
                    "index [{}] should be 0, found [{}]",
                    value, read_buffer[value]
                ),
            }
        }
    }
}
