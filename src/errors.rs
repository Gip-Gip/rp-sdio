//! Errors in the msbaker ecosystem

use embedded_io::{blocking::ReadExactError, Error, ErrorKind};
use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum SdioError {
    #[snafu(display("Timeout on SD command!"))]
    CmdTimeout { },
    #[snafu(display("Timeout on write!"))]
    WriteTimeout {},
    #[snafu(display("Response is to the wrong command!"))]
    WrongCmd { },
    #[snafu(display("Bad rx CRC7!"))]
    BadRxCrc7 {},
    #[snafu(display("Bad tx CRC7!"))]
    BadTxCrc7 {},
    #[snafu(display("Bad rx CRC16!"))]
    BadRxCrc16 { good_crc: u64, bad_crc: u64 },
    #[snafu(display("Bad tx CRC16!"))]
    BadTxCrc16 {},
    #[snafu(display("Bad CMD8 check pattern!"))]
    BadCheck { good_check: u8, bad_check: u8 },
    #[snafu(display("Bad CMD8 voltage!"))]
    BadVoltage { bad_volt: u8 },
    #[snafu(display("Bad OCR voltage range!"))]
    BadVoltRange {},
    #[snafu(display("Failed to write!"))]
    WriteFail {},
    #[snafu(display("Unknown response to write!"))]
    WriteUnknown {},
    #[snafu(display("CC Error!"))]
    CcError {},
    #[snafu(display("Card ECC failed!"))]
    CardEccFailed {},
    #[snafu(display("Illegal Command!"))]
    IllegalCommand {},
    #[snafu(display("Out of range!"))]
    OutOfRange {},
    #[snafu(display("Misaligned address!"))]
    AddressError {},
    #[snafu(display("Block length incorrect!"))]
    BlockLenError {},
    #[snafu(display("Erase sequence error!"))]
    EraseSeqError {},
    #[snafu(display("Erase parameters error!"))]
    EraseParamError {},
    #[snafu(display("Write protection violation!"))]
    WpViolation {},
    #[snafu(display("Card is locked!"))]
    CardIsLocked {},
    #[snafu(display("Failed to lock/unlock card!"))]
    LockUnlockFailed {},
    #[snafu(display("CSD overwite error!"))]
    CsdOverwrite {},
    #[snafu(display("Wp erase skip!"))]
    WpEraseSkip {},
    #[snafu(display("Authentication error!"))]
    AkeSeqError {},
    #[snafu(display("Unknown cmd error!"))]
    CmdUnknownErr {},
    #[snafu(display("SDIO already transferring/recieving! Should definitely not happen!"))]
    InTxRx {},
    #[snafu(display("Unexpected EOF!"))]
    UnexpectedEof {},
}

impl Error for SdioError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl From<SdioError> for ReadExactError<SdioError> {
    fn from(e: SdioError) -> ReadExactError<SdioError> {
        Self::Other(e)
    }
}

impl From<ReadExactError<SdioError>> for SdioError {
    fn from(e: ReadExactError<SdioError>) -> SdioError {
        match e {
            ReadExactError::UnexpectedEof => Self::UnexpectedEof {  },
            ReadExactError::Other(e) => e,
        }
    }
}
