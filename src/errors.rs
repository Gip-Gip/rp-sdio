//! Errors in the msbaker ecosystem

use embedded_io::{blocking::ReadExactError, Error, ErrorKind};
use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum SdioError {
    #[snafu(display("(SDIO) Timeout on SD command!"))]
    SdioCmdTimeout { },
    #[snafu(display("(SDIO) Timeout on write!"))]
    SdioWriteTimeout {},
    #[snafu(display("(SDIO) Response is to the wrong command!"))]
    SdioWrongCmd { },
    #[snafu(display("(SDIO) Bad rx CRC7!"))]
    SdioBadRxCrc7 {},
    #[snafu(display("(SDIO) Bad tx CRC7!"))]
    SdioBadTxCrc7 {},
    #[snafu(display("(SDIO) Bad rx CRC16!"))]
    SdioBadRxCrc16 { good_crc: u64, bad_crc: u64 },
    #[snafu(display("(SDIO) Bad tx CRC16!"))]
    SdioBadTxCrc16 {},
    #[snafu(display("(SDIO) Bad CMD8 check pattern!"))]
    SdioBadCheck { good_check: u8, bad_check: u8 },
    #[snafu(display("(SDIO) Bad CMD8 voltage!"))]
    SdioBadVoltage { bad_volt: u8 },
    #[snafu(display("(SDIO) Bad OCR voltage range!"))]
    SdioBadVoltRange {},
    #[snafu(display("(SDIO) Failed to write!"))]
    SdioWriteFail {},
    #[snafu(display("(SDIO) Unknown response to write!"))]
    SdioWriteUnknown {},
    #[snafu(display("(SDIO) CC Error!"))]
    SdioCcError {},
    #[snafu(display("(SDIO) Card ECC failed!"))]
    SdioCardEccFailed {},
    #[snafu(display("(SDIO) Illegal Command!"))]
    SdioIllegalCommand {},
    #[snafu(display("(SDIO) Out of range!"))]
    SdioOutOfRange {},
    #[snafu(display("(SDIO) Misaligned address!"))]
    SdioAddressError {},
    #[snafu(display("(SDIO) Block length incorrect!"))]
    SdioBlockLenError {},
    #[snafu(display("(SDIO) Erase sequence error!"))]
    SdioEraseSeqError {},
    #[snafu(display("(SDIO) Erase parameters error!"))]
    SdioEraseParamError {},
    #[snafu(display("(SDIO) Write protection violation!"))]
    SdioWpViolation {},
    #[snafu(display("(SDIO) Card is locked!"))]
    SdioCardIsLocked {},
    #[snafu(display("(SDIO) Failed to lock/unlock card!"))]
    SdioLockUnlockFailed {},
    #[snafu(display("(SDIO) CSD overwite error!"))]
    SdioCsdOverwrite {},
    #[snafu(display("(SDIO) Wp erase skip!"))]
    SdioWpEraseSkip {},
    #[snafu(display("(SDIO) Authentication error!"))]
    SdioAkeSeqError {},
    #[snafu(display("(SDIO) Unknown cmd error!"))]
    SdioCmdUnknownErr {},
    #[snafu(display("(PE)SDIO already transferring/recieving! Should definitely not happen!"))]
    SdioInTxRx {},
    #[snafu(display("(IO) Unexpected EOF!"))]
    IoUnexpectedEof {},
    #[snafu(display("Programmer Error!"))]
    PE {},
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
            ReadExactError::UnexpectedEof => Self::IoUnexpectedEof {  },
            ReadExactError::Other(e) => e,
        }
    }
}
