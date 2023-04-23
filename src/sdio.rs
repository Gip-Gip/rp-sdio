//! Code used to interface with SD cards using SDIO

use crate::errors::SdioError;
use crate::registers::{SdCic, SdCid, SdCsd, SdOcr};
use core::{cmp, mem};
use cortex_m::singleton;
use embedded_io::blocking::{Read, Seek, Write};
use embedded_io::{Io, SeekFrom};
use hal::dma::single_buffer;
use hal::dma::single_buffer::Transfer;
use hal::dma::SingleChannel as SingleChannelDma;
use hal::pio::{
    InstalledProgram, MovStatusConfig, PIOBuilder, PIOExt, PinDir, Running, Rx, ShiftDirection,
    StateMachine, Tx, UninitStateMachine, PIO, SM0, SM1,
};
use hal::Timer;
use pio::{Instruction, InstructionOperands, OutDestination, SetDestination};
use pio_proc::pio_file;
use rp2040_hal as hal;

/// Timeout for SD commands, in milliseconds
pub const SD_CMD_TIMEOUT_MS: u64 = 1_000;

/// Startup time for sd card, in milliseconds
pub const SD_STARTUP_MS: u64 = 2;

/// OCR voltage range, set to 3.2-3.4
pub const SD_OCR_VOLT_RANGE: u32 = 0b0011_0000__0000_0000_0000_0000;

/// Block length for SD operations
pub const SD_BLOCK_LEN: usize = 512;

/// Size of the quaduple-crc at the end of a block
pub const SD_BLOCK_CRC_LEN: usize = 8;

/// Size of the end signal at the end of tx transmissions

pub const SD_BLOCK_END_LEN: usize = 4;

/// Size of all tx/rx transfers

pub const SD_BLOCK_LEN_TOTAL: usize = SD_BLOCK_LEN + SD_BLOCK_CRC_LEN + SD_BLOCK_END_LEN;

/// Multiplier for nibbles

pub const SD_NIBBLE_MULT: usize = 2;

/// Divider for words

pub const SD_WORD_DIV: usize = 4;

/// Index of the crc in the working block

pub const SD_CRC_IDX_MSB: usize = SD_BLOCK_LEN / SD_WORD_DIV;
pub const SD_CRC_IDX_LSB: usize = SD_CRC_IDX_MSB + 1;

/// Clock speed divider for initialization transfers
pub const SD_CLK_DIV_INIT: u16 = 25;

/// Retry count
pub const SD_RETRIES: usize = 3;

/// CRC7 Table used for calculating all 7-bit sd-card crcs
pub static CRC7_TABLE: [u8; 256] = [
    0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e, 0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
    0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c, 0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
    0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a, 0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
    0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28, 0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
    0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6, 0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
    0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84, 0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
    0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2, 0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
    0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
    0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc, 0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
    0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce, 0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
    0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98, 0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
    0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa, 0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
    0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34, 0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
    0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06, 0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
    0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50, 0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
    0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62, 0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2,
];

/// Simple function that calculates a crc7 from words.
/// skip specifies how many bytes to skip before the data begins, len specifies
/// the length of the data in bytes.
pub fn calculate_crc7_from_words(words: &[u32], skip: usize, len: usize) -> u8 {
    let mut crc: u8 = 0;

    let start_word = skip / 4; // Words are 32 bits long
    let start_shift = 24 - ((skip % 4) * 8);

    let mut word = start_word;
    let mut shift = start_shift;

    for _ in 0..len {
        // Calculate crc
        let byte = ((words[word] >> shift) & 0xFF) as u8;
        let index = byte ^ crc;
        crc = CRC7_TABLE[index as usize];

        // Advance the shift and/or the word

        if shift == 0 {
            shift = 24;
            word += 1;
        } else {
            shift -= 8;
        }
    }

    crc
}

// This crc algorithm is based on the CRC16 generator/checker specified in the
// Part 1 Physical Layer Simplified Specification
pub fn crc16_4bit(data: &[u32]) -> u64 {
    let mut crc: u64 = 0;

    // Simulate a hardware crc calculator
    for word in data {
        // Convert the data from 4 bytes into 1 word, and assign it to "data in"
        // "data_in" basically represents the input signal to our virtual hardware
        // crc calculator
        let data_in = word;

        // Discard the data that is "getting written out" and use it to xor
        // with the data "coming in"
        let mut data_out: u32 = (crc >> 32) as u32;
        crc <<= 32;

        // "data_out" is only affected by the last XOR in the diagram, which
        // is the last 4 bits of each data line(last 16 of all 4). pre-xor
        // the data_out and data in where it matters
        data_out ^= (data_out ^ data_in) >> 16;

        // xor the data in and the data out, like in the diagram
        let xorred: u64 = (data_out ^ data_in) as u64;

        // then apply it to the different XORs in the diagram(position 0, 5, and 12)
        crc ^= xorred;
        crc ^= xorred << (5 * 4);
        crc ^= xorred << (12 * 4);
    }

    crc
}

const FLAG_ACMD: u16 = 0x40;
const FLAG_R0: u16 = 0x00;
const FLAG_R1: u16 = 0x80;
const FLAG_R1B: u16 = 0x100;
const FLAG_R2: u16 = 0x180;
const FLAG_R3: u16 = 0x200;
const FLAG_R6: u16 = 0x280;
const FLAG_R7: u16 = 0x300;
const FLAGS_R: u16 = 0x380;

#[derive(PartialEq, Clone, Copy)]
#[repr(u16)]
/// Possible commands to give to the SD card
/// **May be macro'd out in future versions**
pub enum SdCmd {
    /// CMD0: GO_IDLE_STATE, R0
    GoIdleState = FLAG_R0 | 0,
    /// CMD2: ALL_SEND_CID, requests all SD cards send their CIDs. R2.
    AllSendCid = FLAG_R2 | 2,
    /// CMD3: SEND_RELATIVE_ADDR, asks the card to send it's RCA. R6.
    SendRelativeAddr = FLAG_R6 | 3,
    // /// !TODO! CMD6: SWITCH_FUNC, R1
    /// CMD7: SELECT_DESELECT_CARD, selects the card if the RCA is that of
    /// the card, deselects it otherwise. Supply the RCA. R1b.
    SelectDeselectCard(u16) = FLAG_R1B | 7,
    /// CMD8: SEND_IF_COND, asks if the sd card supports the current voltage along if
    /// it supports pcie. Supply the check pattern to be verified. R7.
    SendIfCond(u8) = FLAG_R7 | 8,
    /// CMD9: SEND_CSD, asks the card to send it's csd. Supply the RCA
    /// of the card. R2.
    SendCsd(u16) = FLAG_R2 | 9,
    // /// !TODO! CMD10: SEND_CID, R1.
    // /// !TODO! CMD12: STOP_TRANSMISSION, R1b.
    // /// !TODO! CMD13: SEND_STATUS, R2.
    /// CMD16: SET_BLOCKLEN, sets the block length to be used in all reads
    /// and writes. Supply the block length as a u32. R1.
    SetBlockLen(u32) = FLAG_R1 | 16,
    /// CMD17: READ_SINGLE_BLOCK, start a single block read from the
    /// SD card. Supply the data address with this command. R1.
    ReadSingleBlock(u32) = FLAG_R1 | 17,
    // /// !TODO! CMD18: READ_MULTIPLE_BLOCK, R1.
    /// CMD24: WRITE_BLOCK, start a single block write to the SD card. Supply
    /// the data address with this command. R1.
    WriteBlock(u32) = FLAG_R1 | 24,
    // /// !TODO! CMD25: WRITE_MULTIPLE_BLOCK, R1.
    // /// !TODO! CMD27: PROGRAM_CSD, R1.
    // /// !TODO! CMD32: ERASE_WR_BLK_START, R1.
    // /// !TODO! CMD33: ERASE_WR_BLK_END, R1.
    // /// !TODO! CMD38: ERASE, R1b.
    /// CMD55: APP_CMD, R1. **DO NOT USE.** APP_CMD is automatically sent before any ACMD.
    /// Supply the RCA with this command
    AppCmd(u16) = FLAG_R1 | 55,
    // /// !TODO! CMD56: GEN_CMD
    /// ACMD6: SET_BUS_WIDTH, sets the bus width to 4-bit if the supplied boolean
    /// is true, 1-bit if false. R1
    SetBusWidth(bool) = FLAG_R1 | FLAG_ACMD | 6,
    // /// !TODO! ACMD13: SD_STATUS
    // /// !TODO! ACMD22: SEND_NUM_WR_BLOCKS
    // /// !TODO! ACMD23: SET_WR_BLK_ERASE_COUNT
    /// ACMD41: SD_APP_OP_COND, Sends host capacity support info and asks the card to send the OCR
    /// Supply HCS, XPC, S18R, and Voltage Window. R3
    SdAppOpCond(bool, bool, bool, u32) = FLAG_R3 | FLAG_ACMD | 41,
    // /// !TODO! ACMD42: SET_CLR_CARD_DETECT
    // /// !TODO! ACMD51: SEND_SCR
}

impl Into<u16> for SdCmd {
    fn into(self) -> u16 {
        // SAFETY: the enum is `#[repr(u16)]`
        unsafe { (&self as *const Self).cast::<u16>().read() }
    }
}

impl SdCmd {
    #[inline]
    /// Get the index of a command
    pub fn get_cmd_index(&self) -> u32 {
        let val: u16 = (*self).into();

        // The first 6 bits is the command index
        (val & 0x3f) as u32
    }

    /// Get the response type of a command
    pub fn get_cmd_response(&self) -> SdCmdResponseType {
        let val: u16 = (*self).into();

        match val & FLAGS_R {
            FLAG_R1 => SdCmdResponseType::R1,
            FLAG_R1B => SdCmdResponseType::R1b,
            FLAG_R2 => SdCmdResponseType::R2,
            FLAG_R3 => SdCmdResponseType::R3,
            FLAG_R6 => SdCmdResponseType::R6,
            FLAG_R7 => SdCmdResponseType::R7,
            _ => SdCmdResponseType::R0,
        }
    }

    #[inline]
    /// Returns true if a command is an app command
    pub fn is_acmd(&self) -> bool {
        let val: u16 = (*self).into();

        (val & FLAG_ACMD) != 0
    }

    /// Format command as 2 words. For use with send_command
    pub fn format(&self) -> [u32; 2] {
        // Bits 63-56 are the length of the command in bits, -1
        // Always 47, but specified because PIO asm can't autonomously set it's
        // internal registers to anything above 31
        //
        // Bits 55-54 will always be 0b01, as it specifies the starting bit(0) and the command
        // direction(host->card)
        //
        // Bit 8 is always 1(end bit)
        let mut data: [u32; 2] = [
            0b0010_1111_0100_0000__0000_0000_0000_0000,
            0b0000_0000_0000_0000__0000_0001_0000_0000,
        ];

        // Bits 53-48 is the command index
        data[0] |= self.get_cmd_index() << 16;

        // Bits 47-16 is the argument
        // Sort this match case by most commonly used first
        // May macro out this entire code block
        match self {
            Self::SetBlockLen(unsigned32)
            | Self::ReadSingleBlock(unsigned32)
            | Self::WriteBlock(unsigned32) => {
                data[0] |= (unsigned32 >> 16) & 0xFFFF;
                data[1] |= (unsigned32 << 16) & 0xFFFF_0000;
            }
            // And some commands don't have an argument
            Self::GoIdleState | Self::AllSendCid | Self::SendRelativeAddr => {}
            Self::SelectDeselectCard(rca) | Self::SendCsd(rca) | Self::AppCmd(rca) => {
                // Bits 47-32 are the RCA, 31-16 are stuff bits
                data[0] |= *rca as u32;
            }
            Self::SetBusWidth(fourbit) => {
                data[1] |= match fourbit {
                    true => 0b10 << 16,
                    false => 0,
                }
            }
            Self::SendIfCond(test_pattern) => {
                // Bits 47-28 are reserved
                // 27-24 is the supply voltage, however always 0b0001 (2.7-3.6v)
                // both because that is the rp2040 voltage and because no other
                // ranges are specified
                data[1] |= 0b0001 << 24;
                // 23-16 is the check pattern
                data[1] |= (*test_pattern as u32) << 16;
            }
            Self::SdAppOpCond(hcs, xpc, s18r, voltage_window) => {
                // Bit 47 is reserved
                // Bit 46 is the HCS
                if *hcs {
                    data[0] |= 1 << 14;
                }
                // Bit 45 is reserved
                // Bit 44 is XPC
                if *xpc {
                    data[0] |= 1 << 12;
                }
                // Bits 43-41 are reserved
                // Bit 40 is S18R
                if *s18r {
                    data[0] |= 1 << 8;
                }

                // Bits 39-16 is the Voltage Window
                data[0] |= (voltage_window >> 16) & 0xFF;
                data[1] |= (voltage_window << 16) & 0xFFFF_0000;
            }
        }

        // Bits 15-9 is the crc7 of the command, processed in big endian
        let crc = calculate_crc7_from_words(&data, 1, 5);
        data[1] |= (crc as u32) << 8;

        // Bits 7-0 is the length of the response to the command - 1
        let response_len = self.get_cmd_response().get_response_len() as u32;

        if response_len > 0 {
            data[1] |= response_len - 1;
        }

        data
    }
}

/// Possible response types for SD commands
#[derive(PartialEq)]
pub enum SdCmdResponseType {
    /// R0, no response
    R0,
    /// R1, normal response
    R1,
    /// R1b, normal response with busyx_dm
    R1b,
    /// R2, CID/CSD register
    R2,
    /// R3, OCR register
    R3,
    /// R6, Published RCA response
    R6,
    /// R7, Card interface condition
    R7,
}

impl SdCmdResponseType {
    /// Get the length of the response in bits
    pub fn get_response_len(&self) -> u8 {
        match self {
            SdCmdResponseType::R0 => 0,
            SdCmdResponseType::R1
            | SdCmdResponseType::R1b
            | SdCmdResponseType::R3
            | SdCmdResponseType::R6
            | SdCmdResponseType::R7 => 48,
            SdCmdResponseType::R2 => 136,
        }
    }
}

/// Same as SdCmdResponseType, just for returning the actual values
pub enum SdCmdResponse {
    /// R0, returns nothing
    R0,
    /// R1, returns the card status
    R1(SdCardStatus),
    /// R1b, identical to R1
    R1b(SdCardStatus),
    /// R2, returns cid or csd
    R2([u32; 4]),
    /// R3, returns OCR register
    R3(SdOcr),
    /// R6, returns RCA
    R6(u16),
    /// R7, returns CIC
    R7(SdCic),
}

/// Status of the sd card
pub struct SdCardStatus {
    pub card_status: u32,
}

impl SdCardStatus {
    /// Gets the current state of the card
    pub fn get_current_state(&self) -> SdCurrentState {
        let current_state = (self.card_status >> 9) & 0xF;

        SdCurrentState::from_int(current_state as u8)
    }

    pub fn get_worst_error(&self) -> Result<(), SdioError> {
        // First see if there is an error
        if (self.card_status & 0b1111_1101_1111_1001__1000_0000_0000_1000) == 0 {
            return Ok(());
        }

        // Proceed with the if statements...
        // !TODO! make less ugly
        if self.cc_error() {
            return Err(SdioError::CcError {});
        }

        if self.com_crc_error() {
            return Err(SdioError::BadTxCrc7 {});
        }

        if self.illegal_command() {
            return Err(SdioError::IllegalCommand {});
        }

        if self.card_ecc_failed() {
            return Err(SdioError::CardEccFailed {});
        }

        if self.out_of_range() {
            return Err(SdioError::OutOfRange {});
        }

        if self.address_error() {
            return Err(SdioError::AddressError {});
        }

        if self.block_len_error() {
            return Err(SdioError::BlockLenError {});
        }

        if self.erase_seq_error() {
            return Err(SdioError::EraseSeqError {});
        }

        if self.erase_param() {
            return Err(SdioError::EraseParamError {});
        }

        if self.wp_violation() {
            return Err(SdioError::WpViolation {});
        }

        if self.lock_unlocked_failed() {
            return Err(SdioError::LockUnlockFailed {});
        }

        if self.wp_erase_skip() {
            return Err(SdioError::WpEraseSkip {});
        }

        if self.ake_seq_error() {
            return Err(SdioError::AkeSeqError {});
        }

        Err(SdioError::CmdUnknownErr {})
    }

    #[inline]
    /// Returns true if the OUT_OF_RANGE bit is set
    pub fn out_of_range(&self) -> bool {
        (self.card_status & (1 << 31)) > 0
    }
    #[inline]
    /// Returns true if the ADDRESS_ERROR bit is set
    pub fn address_error(&self) -> bool {
        (self.card_status & (1 << 30)) > 0
    }
    #[inline]
    /// Returns true if the BLOCK_LEN_ERROR bit is set
    pub fn block_len_error(&self) -> bool {
        (self.card_status & (1 << 29)) > 0
    }
    #[inline]
    /// Returns true if the ERASE_SEQ_ERROR bit is set
    pub fn erase_seq_error(&self) -> bool {
        (self.card_status & (1 << 28)) > 0
    }
    #[inline]
    /// Returns true if the ERASE_PARAM bit is set
    pub fn erase_param(&self) -> bool {
        (self.card_status & (1 << 27)) > 0
    }
    #[inline]
    /// Returns true if the WP_VIOLATION bit is set
    pub fn wp_violation(&self) -> bool {
        (self.card_status & (1 << 26)) > 0
    }
    #[inline]
    /// Returns true if the CARD_IS_LOCKED bit is set
    pub fn card_is_locked(&self) -> bool {
        (self.card_status & (1 << 25)) > 0
    }
    #[inline]
    /// Returns true if the LOCK_UNLOCK_FAILED bit is set
    pub fn lock_unlocked_failed(&self) -> bool {
        (self.card_status & (1 << 24)) > 0
    }
    #[inline]
    /// Returns true if the COM_CRC_ERROR bit is set
    pub fn com_crc_error(&self) -> bool {
        (self.card_status & (1 << 23)) > 0
    }
    #[inline]
    /// Returns true if the ILLEGAL_COMMAND bit is set
    pub fn illegal_command(&self) -> bool {
        (self.card_status & (1 << 22)) > 0
    }
    #[inline]
    /// Returns true if the CARD_ECC_FAILED bit is set
    pub fn card_ecc_failed(&self) -> bool {
        (self.card_status & (1 << 21)) > 0
    }
    #[inline]
    /// Returns true if the CC_ERROR bit is set
    pub fn cc_error(&self) -> bool {
        (self.card_status & (1 << 20)) > 0
    }
    #[inline]
    /// Returns true if the ERROR bit is set
    pub fn error(&self) -> bool {
        (self.card_status & (1 << 19)) > 0
    }
    #[inline]
    /// Returns true if the CSD_OVERWRITE bit is set
    pub fn csd_overwrite(&self) -> bool {
        (self.card_status & (1 << 16)) > 0
    }
    #[inline]
    /// Returns true if the WP_ERASE_SKIP bit is set
    pub fn wp_erase_skip(&self) -> bool {
        (self.card_status & (1 << 15)) > 0
    }
    #[inline]
    /// Returns true if the CARD_ECC_DISABLED bit is set
    pub fn card_ecc_disabled(&self) -> bool {
        (self.card_status & (1 << 14)) > 0
    }
    #[inline]
    /// Returns true if the ERASE_RESET bit is set
    pub fn erase_reset(&self) -> bool {
        (self.card_status & (1 << 13)) > 0
    }
    #[inline]
    /// Returns true if the READY_FOR_DATA bit is set
    pub fn ready_for_data(&self) -> bool {
        (self.card_status & (1 << 8)) > 0
    }
    #[inline]
    /// Returns true if the FX_EVENT bit is set
    pub fn fx_event(&self) -> bool {
        (self.card_status & (1 << 6)) > 0
    }
    #[inline]
    /// Returns true if the APP_CMD bit is set
    pub fn app_cmd(&self) -> bool {
        (self.card_status & (1 << 5)) > 0
    }
    #[inline]
    /// Returns true if the AKE_SEQ_ERROR bit is set
    pub fn ake_seq_error(&self) -> bool {
        (self.card_status & (1 << 3)) > 0
    }
}

pub enum SdCurrentState {
    Idle,
    Ready,
    Ident,
    Stby,
    Tran,
    Data,
    Rcv,
    Prg,
    Dis,
    Reserved,
}

impl SdCurrentState {
    /// Grabs the sd state from a nibble
    pub fn from_int(nibble: u8) -> Self {
        match nibble {
            0 => Self::Idle,
            1 => Self::Ready,
            2 => Self::Ident,
            3 => Self::Stby,
            4 => Self::Tran,
            5 => Self::Data,
            6 => Self::Rcv,
            7 => Self::Prg,
            8 => Self::Dis,
            _ => Self::Reserved,
        }
    }
}

/// SDIO 4bit interface struct
pub struct Sdio4bit<'a, DmaCh: SingleChannelDma, P: PIOExt> {
    /// DMA used to read/write data from the SD card
    dma: Option<DmaCh>,
    /// Reference to the processor's timer
    timer: &'a Timer,
    /// The command state machine, controls SD_CLK and SD_CMD
    sm_cmd: StateMachine<(P, SM0), Running>,
    sm_cmd_rx: Rx<(P, SM0)>,
    sm_cmd_tx: Tx<(P, SM0)>,
    /// The data state machine, controls SD_DAT0-SD_DAT3
    /// Is an option since it can be used up during a tx or an rx
    sm_dat: Option<UninitStateMachine<(P, SM1)>>,
    /// The base pin for the data lines, aka DAT0
    sd_dat_base_id: u8,
    /// The data recieve program
    program_data_rx: Option<InstalledProgram<P>>,
    /// The data transmit program
    /// Is an option since it can be used up during a tx
    program_data_tx: Option<InstalledProgram<P>>,

    /// The CID register for the card
    cid: SdCid,
    /// The RCA register for the card
    rca: u16,
    /// The CSD register for the card
    csd: SdCsd,

    /// The current working block for the SD card
    working_block: Option<&'static mut [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV]>,
    /// The address of the current working block
    working_block_num: u32,
    /// The current read/write position
    position: u64,
    /// The size of the sd card in blocks
    size: u32,

    /// Temporary home for Rx and Tx not used by DMAs
    sm_dat_rx: Option<Rx<(P, SM1)>>,
    sm_dat_tx: Option<Tx<(P, SM1)>>,

    /// Home for DMA during TX
    tx_dma_in_use:
        Option<Transfer<DmaCh, &'static mut [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV], Tx<(P, SM1)>>>,
    /// Home for DMA during RX
    rx_dma_in_use:
        Option<Transfer<DmaCh, Rx<(P, SM1)>, &'static mut [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV]>>,
    /// Home for the cmd state machine during TX and RX
    sm_in_use: Option<StateMachine<(P, SM1), Running>>,

    /// Clock divider after initialization
    sd_full_clk_div: u16,
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt> Sdio4bit<'a, DmaCh, P> {
    /// Create a new SDIO interface
    pub fn new(
        pio: &mut PIO<P>,
        dma: DmaCh,
        timer: &'a Timer,
        sm0: UninitStateMachine<(P, SM0)>,
        sm1: UninitStateMachine<(P, SM1)>,
        sd_clk_id: u8,
        sd_cmd_id: u8,
        sd_dat_base_id: u8,
        sd_full_clk_div: u16,
    ) -> Self {
        // Initialilze the raw program variables from the rp2040_sdio.pio file
        let program_cmd_clk =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_cmd_clk")).program;
        let program_data_rx =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_rx")).program;
        let program_data_tx =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_tx")).program;

        // Install them to the pio
        let program_cmd_clk = pio.install(&program_cmd_clk).unwrap();
        let program_data_rx = pio.install(&program_data_rx).unwrap();
        let program_data_tx = pio.install(&program_data_tx).unwrap();

        let (mut sm_cmd, sm_cmd_rx, sm_cmd_tx) = PIOBuilder::from_program(program_cmd_clk)
            .set_mov_status_config(MovStatusConfig::Tx(2))
            .set_pins(sd_cmd_id, 1)
            .out_pins(sd_cmd_id, 1)
            .in_pin_base(sd_cmd_id)
            .jmp_pin(sd_cmd_id)
            .side_set_pin_base(sd_clk_id)
            .out_shift_direction(ShiftDirection::Left)
            .in_shift_direction(ShiftDirection::Left)
            // Set the initial clock speed to ~1mhz for initialization
            .clock_divisor_fixed_point(SD_CLK_DIV_INIT, 0)
            .autopush(true)
            .autopull(true)
            .build(sm0);

        let working_block = singleton!(: [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV] = [0xAF; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV]).unwrap();

        sm_cmd.set_pindirs([(sd_clk_id, PinDir::Output), (sd_cmd_id, PinDir::Output)]);
        // Start the state machine
        let sm_cmd = sm_cmd.start();

        let sm_dat = sm1;

        Self {
            dma: Some(dma),
            timer,
            sm_cmd,
            sm_cmd_rx,
            sm_cmd_tx,
            sm_dat: Some(sm_dat),
            sd_dat_base_id,
            program_data_rx: Some(program_data_rx),
            program_data_tx: Some(program_data_tx),
            cid: SdCid::new([0; 4]),
            // RCA must initially be 0 for the first CMD55 to work properly
            rca: 0,
            csd: SdCsd::new([0; 4]),

            working_block: Some(working_block),
            working_block_num: 0,
            position: 0,
            size: 0,

            // All start as none
            sm_dat_rx: None,
            sm_dat_tx: None,
            tx_dma_in_use: None,
            rx_dma_in_use: None,
            sm_in_use: None,
            sd_full_clk_div,
        }
    }

    /// Send a command to the SD card, retries thrice
    pub fn send_command(&mut self, command: SdCmd) -> Result<SdCmdResponse, SdioError> {
        for _ in 1..SD_RETRIES {
            match self.send_command_raw(command) {
                Ok(response) => {
                    return Ok(response);
                }
                Err(_) => {}
            }
        }

        self.send_command_raw(command)
    }

    /// Send a command to the SD card, does not retry
    fn send_command_raw(&mut self, command: SdCmd) -> Result<SdCmdResponse, SdioError> {
        // If the command is an app command, first send the AppCmd command
        if command.is_acmd() {
            self.send_command(SdCmd::AppCmd(self.rca))?;
        }

        // Get the data for the command
        let command_data = command.format();

        // Push both words into tx
        self.sm_cmd_tx.write(command_data[0]);
        self.sm_cmd_tx.write(command_data[1]);

        let response_type = command.get_cmd_response();

        // Create a buffer and make the actually accessable portion equal to
        // the response size. Max size is R2, which is 136 bits/17 bytes/4.25 wqrds
        let mut resp_buf: [u32; 5] = [0; 5];

        // Calculate the length of the response, and add 1 since all response lengths are not
        // divisable by 32 bits
        let resp_len_bits = response_type.get_response_len();
        let resp_len: usize = ((resp_len_bits / 32) + 1) as usize;

        for i in 0..resp_len {
            let start_time = self.timer.get_counter();
            while self.sm_cmd_rx.is_empty() {
                let current_time = self.timer.get_counter();

                let time_delta = current_time
                    .checked_duration_since(start_time)
                    .unwrap()
                    .to_millis();

                if time_delta > SD_CMD_TIMEOUT_MS {
                    return Err(SdioError::CmdTimeout {
                        cmd: command.get_cmd_index() as u8,
                        time_ms: time_delta,
                    });
                }
            }

            resp_buf[i] = self.sm_cmd_rx.read().unwrap();
        }

        // shift the last word so the same crc calculation can be used for in
        // and out data
        resp_buf[resp_len - 1] = resp_buf[resp_len - 1] << (32 - (resp_len_bits % 32));

        // Return here if no response, otherwise get the response
        if response_type == SdCmdResponseType::R0 {
            return Ok(SdCmdResponse::R0);
        }

        // Verifty the command index and crc (if it has one)
        if response_type != SdCmdResponseType::R2 && response_type != SdCmdResponseType::R3 {
            // All commands with a CRC are 48 bits, with the last 8 bits being the CRC7 + stop bit

            // Calculate the CRC up to the CRC
            let good_crc = calculate_crc7_from_words(&resp_buf, 0, 5);

            let crc = ((resp_buf[1] >> 16) & 0xFE) as u8;

            if good_crc != crc {
                return Err(SdioError::BadRxCrc7 {
                    good_crc,
                    bad_crc: crc,
                });
            }

            let good_command_index = command.get_cmd_index();

            let command_index = (resp_buf[0] >> 24) & 0x3F;

            if good_command_index != command_index {
                return Err(SdioError::WrongCmd {
                    good_cmd: good_command_index as u8,
                    bad_cmd: command_index as u8,
                });
            }
        }

        // Construct the response
        match response_type {
            SdCmdResponseType::R1 => {
                let card_status = SdCardStatus {
                    card_status: ((resp_buf[0] & 0x00FF_FFFF) << 8)
                        | ((resp_buf[1] & 0xFF00_0000) >> 24),
                };

                // Make sure we didn't get any errors
                card_status.get_worst_error()?;

                Ok(SdCmdResponse::R1(card_status))
            }
            SdCmdResponseType::R1b => {
                let card_status = SdCardStatus {
                    card_status: ((resp_buf[0] & 0x00FF_FFFF) << 8)
                        | ((resp_buf[1] & 0xFF00_0000) >> 24),
                };

                card_status.get_worst_error()?;

                Ok(SdCmdResponse::R1b(card_status))
            }
            SdCmdResponseType::R2 => {
                let mut words: [u32; 4] = [0; 4];

                // Shift all the data left 8 bits to remove the first 8 bits of
                // the response
                words[0] = ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                words[1] = ((resp_buf[1] & 0x00FF_FFFF) << 8) | ((resp_buf[2] & 0xFF00_0000) >> 24);
                words[2] = ((resp_buf[2] & 0x00FF_FFFF) << 8) | ((resp_buf[3] & 0xFF00_0000) >> 24);
                words[3] = ((resp_buf[3] & 0x00FF_FFFF) << 8) | ((resp_buf[4] & 0xFF00_0000) >> 24);

                Ok(SdCmdResponse::R2(words))
            }
            SdCmdResponseType::R3 => {
                let ocr = ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);

                Ok(SdCmdResponse::R3(SdOcr { ocr }))
            }
            SdCmdResponseType::R6 => {
                let rca: u16 = ((resp_buf[0] & 0x00FF_FF00) >> 8) as u16;

                // You also get some status bits with this command, but I doubt
                // there is a single scenario where it is useful to read them.
                //
                // Feel free to correct me though.
                Ok(SdCmdResponse::R6(rca))
            }
            SdCmdResponseType::R7 => {
                let good_check_pattern = match command {
                    SdCmd::SendIfCond(check_pattern) => check_pattern,
                    _ => panic!("Command repsonds with R7, not SendIfCond!"),
                };
                // Bit 21 is the 1.2v support bit, 20 is the pcie support
                //                                   4444 4444 3333 3333  3322 2222 2222 1111
                //                                   7654 3210 9876 5432  1098 7654 3210 9876
                let supports_1p2v = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0010_0000) > 0;
                let supports_pcie = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0001_0000) > 0;

                // Verify the correct voltage is in use
                let voltage = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0000_1111) as u8;

                if voltage != 0b0001 {
                    return Err(SdioError::BadVoltage { bad_volt: voltage });
                }

                // Verify the check pattern
                let check_pattern = ((resp_buf[1] >> 24) & 0xFF) as u8;

                if check_pattern != good_check_pattern {
                    return Err(SdioError::BadCheck {
                        good_check: good_check_pattern,
                        bad_check: check_pattern,
                    });
                }

                Ok(SdCmdResponse::R7(SdCic {
                    supports_1p2v,
                    supports_pcie,
                }))
            }
            SdCmdResponseType::R0 => {
                panic!("Reached R0 somehow!");
            } // Shouldn't happen
        }
    }

    /// Write the block to the SD card
    pub fn start_block_tx(&mut self) -> Result<(), SdioError> {
        // If this value is None, there is already a transfer occuring
        let sm_dat = match mem::take(&mut self.sm_dat) {
            Some(sm) => sm,

            None => {
                return Err(SdioError::InTxRx {});
            }
        };

        // If these values are None, there is already a transfer occuring but
        // for some reason the sm_dat didn't get used up. Something that
        // shouldn't happen, and an unwrap is appropriate here I think.
        let dma = mem::take(&mut self.dma).unwrap();
        let program = mem::take(&mut self.program_data_tx).unwrap();
        let working_block = mem::take(&mut self.working_block).unwrap();

        // Calculate the crc of the block and place it at the end of the block
        let crc = crc16_4bit(&working_block[..SD_BLOCK_LEN / SD_WORD_DIV]);

        let len = working_block.len();

        working_block[SD_CRC_IDX_LSB] = (crc & 0xFFFF_FFFF) as u32;
        working_block[SD_CRC_IDX_MSB] = ((crc >> 32) & 0xFFFF_FFFF) as u32;

        // Place the transimssion end signal at the end of the block
        working_block[len - 1] = 0xFFFF_FFFF;

        // Now initialize the state machine with the TX program
        let (mut sm_dat, sm_dat_rx, mut sm_dat_tx) = PIOBuilder::from_program(program)
            .in_pin_base(self.sd_dat_base_id)
            .set_pins(self.sd_dat_base_id, 4)
            .out_pins(self.sd_dat_base_id, 4)
            .in_shift_direction(ShiftDirection::Left)
            .out_shift_direction(ShiftDirection::Left)
            .autopush(false)
            .autopull(true)
            .clock_divisor_fixed_point(self.sd_full_clk_div, 0)
            .build(sm_dat);

        // Manually store the nibble count and response bit count to X an Y
        sm_dat_tx.write((SD_BLOCK_LEN_TOTAL * SD_NIBBLE_MULT) as u32);
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::OUT {
                destination: OutDestination::X,
                bit_count: 32,
            },
            delay: 0,
            side_set: None,
        });

        sm_dat_tx.write(31);
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::OUT {
                destination: OutDestination::Y,
                bit_count: 32,
            },
            delay: 0,
            side_set: None,
        });

        // Initialize the pins and output to high
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::SET {
                destination: SetDestination::PINS,
                data: 0xF,
            },
            delay: 0,
            side_set: None,
        });

        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::SET {
                destination: SetDestination::PINDIRS,
                data: 0xF,
            },
            delay: 0,
            side_set: None,
        });

        // prep the state machine with the start token
        sm_dat_tx.write(0b1111_1111_1111_1111__1111_1111_1111_0000);

        // Initialize the DMA transfer
        let tx_transfer = single_buffer::Config::new(dma, working_block, sm_dat_tx);

        // Send the write block command
        self.send_command(SdCmd::WriteBlock(self.working_block_num))?;

        // Start the state machine and DMA
        let tx_transfer = tx_transfer.start();
        let sm_dat = sm_dat.start();

        // Store everything needed in the future
        self.tx_dma_in_use = Some(tx_transfer);
        self.sm_in_use = Some(sm_dat);
        self.sm_dat_rx = Some(sm_dat_rx);

        Ok(())
    }

    /// Check the checks the response of the SD card to make sure there
    /// haven't been any errors in writing. Will throw an error if one
    /// is detected.
    pub fn check_write_response(mut response: u32) -> Result<(), SdioError> {
        if (!response & 0xFFFF_0000) == 0 {
            response <<= 16;
        }

        if (!response & 0xFF00_0000) == 0 {
            response <<= 8;
        }

        if (!response & 0xF000_0000) == 0 {
            response <<= 4;
        }

        if (!response & 0xC000_0000) == 0 {
            response <<= 2;
        }

        if (!response & 0x8000_0000) == 0 {
            response <<= 1;
        }

        response = response >> 28 & 7;

        match response {
            2 => Ok(()),
            5 => Err(SdioError::BadTxCrc16 {}),
            6 => Err(SdioError::WriteFail {}),
            _ => Err(SdioError::WriteUnknown { response }),
        }
    }

    #[inline]
    /// Returns true if you need to poll the tx
    pub fn is_tx(&self) -> bool {
        match &self.tx_dma_in_use {
            Some(_) => true,
            None => false,
        }
    }

    /// Poll the current transfer, return true if still transferring
    pub fn poll_tx(&mut self) -> Result<bool, SdioError> {
        // Return false if we don't need to poll tx, true if the DMA is still transferring
        let ready = match &self.tx_dma_in_use {
            Some(tx_dma) => tx_dma.is_done(),
            None => return Ok(false),
        };

        if !ready {
            return Ok(true);
        }

        // Wait until there is something in the fifo
        let start_time = self.timer.get_counter();
        let mut time_delta = 0;
        while self.sm_dat_rx.as_ref().unwrap().is_empty() {
            let current_time = self.timer.get_counter();

            time_delta = current_time
                .checked_duration_since(start_time)
                .unwrap()
                .to_millis();

            if time_delta > SD_CMD_TIMEOUT_MS {
                break;
            }
        }

        // The RX should be there. If not, we'll throw the timeout error after
        // we clean up
        let response = self.sm_dat_rx.as_mut().unwrap().read();

        // Put away everything we used
        // Deconstruct the DMA
        let (dma, working_block, sm_dat_tx) = mem::take(&mut self.tx_dma_in_use).unwrap().wait();

        // Deconstruct the state machine
        let sm_dat_rx = mem::take(&mut self.sm_dat_rx).unwrap();
        let (sm_dat, program_data_tx) = mem::take(&mut self.sm_in_use)
            .unwrap()
            .uninit(sm_dat_rx, sm_dat_tx);

        // Replace the appropriate borrowed variables
        self.program_data_tx = Some(program_data_tx);
        self.dma = Some(dma);
        self.working_block = Some(working_block);
        self.sm_dat = Some(sm_dat);

        let response = match response {
            Some(response) => response,
            None => {
                return Err(SdioError::WriteTimeout {
                    time_ms: time_delta,
                });
            }
        };

        // Check the status of the response and throw errors if appropriate
        Self::check_write_response(response)?;

        // Return false since we are no longer transferring!
        Ok(false)
    }

    #[inline]
    /// Wait for the data transfer to finish
    pub fn wait_tx(&mut self) -> Result<(), SdioError> {
        while self.poll_tx()? {}

        Ok(())
    }

    #[inline]
    /// Start reading data from the SD card into the working block
    pub fn start_block_rx(&mut self) -> Result<(), SdioError> {
        // If this value is None, there is already a transfer occuring
        let sm_dat = match mem::take(&mut self.sm_dat) {
            Some(sm) => sm,

            None => {
                return Err(SdioError::InTxRx {});
            }
        };

        // If these values are None, there is already a transfer occuring but
        // for some reason the sm_dat didn't get used up. Something that
        // shouldn't happen, and an unwrap is appropriate here I think.
        let dma = mem::take(&mut self.dma).unwrap();
        let program = mem::take(&mut self.program_data_rx).unwrap();
        let working_block = mem::take(&mut self.working_block).unwrap();

        // Now initialize the state machine with the RX program
        let (mut sm_dat, sm_dat_rx, mut sm_dat_tx) = PIOBuilder::from_program(program)
            .in_pin_base(self.sd_dat_base_id)
            .set_pins(self.sd_dat_base_id, 4)
            .out_pins(self.sd_dat_base_id, 4)
            .in_shift_direction(ShiftDirection::Left)
            .autopush(true)
            .autopull(true)
            .clock_divisor_fixed_point(self.sd_full_clk_div, 0)
            .build(sm_dat);

        // Manually store the nibble count to X
        sm_dat_tx.write((SD_BLOCK_LEN_TOTAL * SD_NIBBLE_MULT) as u32);
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::OUT {
                destination: OutDestination::X,
                bit_count: 32,
            },
            delay: 0,
            side_set: None,
        });

        // Initialize the pins and output to high (not setting the pins to high
        // can cause errors in testing...)
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::SET {
                destination: SetDestination::PINS,
                data: 0xF,
            },
            delay: 0,
            side_set: None,
        });

        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::SET {
                destination: SetDestination::PINDIRS,
                data: 0xF,
            },
            delay: 0,
            side_set: None,
        });

        // Initialize the pins to input
        sm_dat.exec_instruction(Instruction {
            operands: InstructionOperands::SET {
                destination: SetDestination::PINDIRS,
                data: 0x0,
            },
            delay: 0,
            side_set: None,
        });

        // Initialize the DMA transfer
        let rx_transfer = single_buffer::Config::new(dma, sm_dat_rx, working_block);

        // Start the state machine and DMA
        let rx_transfer = rx_transfer.start();
        let sm_dat = sm_dat.start();

        // Send the read block command
        self.send_command(SdCmd::ReadSingleBlock(self.working_block_num))?;

        // Store everything needed in the future
        self.rx_dma_in_use = Some(rx_transfer);
        self.sm_in_use = Some(sm_dat);
        self.sm_dat_tx = Some(sm_dat_tx);

        Ok(())
    }

    #[inline]
    /// Return true if you need to poll rx
    pub fn is_rx(&self) -> bool {
        match &self.rx_dma_in_use {
            Some(_) => true,
            None => false,
        }
    }

    /// Poll the current transfer, return true if still transferring
    pub fn poll_rx(&mut self) -> Result<bool, SdioError> {
        // Return false if we don't need to poll rx, true if the DMA is still transferring
        let ready = match &self.rx_dma_in_use {
            Some(tx_dma) => tx_dma.is_done(),
            None => return Ok(false),
        };

        if !ready {
            return Ok(true);
        }

        // Put away everything we used
        // Deconstruct the DMA
        let (dma, sm_dat_rx, working_block) = mem::take(&mut self.rx_dma_in_use).unwrap().wait();

        // Deconstruct the state machine
        let sm_dat_tx = mem::take(&mut self.sm_dat_tx).unwrap();
        let (sm_dat, program_data_rx) = mem::take(&mut self.sm_in_use)
            .unwrap()
            .uninit(sm_dat_rx, sm_dat_tx);

        // Save the CRC for later

        let crc: u64 =
            (working_block[SD_CRC_IDX_LSB] as u64) | ((working_block[SD_CRC_IDX_MSB] as u64) << 32);
        let good_crc = crc16_4bit(&working_block[..SD_BLOCK_LEN / SD_WORD_DIV]);

        // Replace the appropriate borrowed variables
        self.program_data_rx = Some(program_data_rx);
        self.dma = Some(dma);
        self.working_block = Some(working_block);
        self.sm_dat = Some(sm_dat);

        // Check the crc and if it's bad, error out!

        if crc != good_crc {
            return Err(SdioError::BadRxCrc16 {
                good_crc,
                bad_crc: crc,
            });
        }

        // Return false since we are no longer transferring!
        Ok(false)
    }

    #[inline]
    /// Wait until we're done recieving data
    pub fn wait_rx(&mut self) -> Result<(), SdioError> {
        while self.poll_rx()? {}

        Ok(())
    }

    /// Returns true if you need to poll rx or tx
    pub fn is_rx_tx(&self) -> bool {
        self.is_rx() | self.is_tx()
    }

    /// Polls both RX and TX, when we don't know which one needs to be polled
    pub fn poll_rx_tx(&mut self) -> Result<bool, SdioError> {
        // If the tx dma or rx dma is Some, call the appropriate poll function
        if let Some(_) = self.tx_dma_in_use {
            return self.poll_tx();
        }

        if let Some(_) = self.rx_dma_in_use {
            return self.poll_rx();
        }

        // Otherwise, neither are in use
        Ok(false)
    }

    /// Waits until the card is neither recieving nor transferring data
    pub fn wait_rx_tx(&mut self) -> Result<(), SdioError> {
        while self.poll_rx_tx()? {}

        Ok(())
    }

    /// Initialize the SD card
    pub fn init(&mut self) -> Result<(), SdioError> {
        // Wait 1ms to ensure that the card is properly initialized
        let start_time = self.timer.get_counter();
        let mut current_time = start_time;
        while current_time
            .checked_duration_since(start_time)
            .unwrap()
            .to_millis()
            < SD_STARTUP_MS
        {
            current_time = self.timer.get_counter();
        }

        // Send CMD0
        self.send_command(SdCmd::GoIdleState)?;

        // Send CMD8
        // !TODO! less runtime checks
        let cic = match self.send_command(SdCmd::SendIfCond(0xAA))? {
            SdCmdResponse::R7(cic) => cic,
            _ => {
                panic!("Incorrect response type!");
            }
        };

        // Send ACMD41 until the card is no longer busy, and once ready verify
        // the voltage window is valid
        let acmd41 = SdCmd::SdAppOpCond(true, true, false, SD_OCR_VOLT_RANGE);

        let mut is_busy = true;
        let mut ocr = SdOcr { ocr: 0 };

        while is_busy {
            ocr = match self.send_command(acmd41)? {
                SdCmdResponse::R3(ocr) => ocr,
                _ => {
                    panic!("Incorrect response type!");
                }
            };

            is_busy = ocr.is_busy();
        }

        let voltage_range = ocr.get_voltage_window();
        if (voltage_range & SD_OCR_VOLT_RANGE) != SD_OCR_VOLT_RANGE {
            return Err(SdioError::BadVoltRange {
                range: voltage_range,
            });
        }

        // Get the CID
        let cid = match self.send_command(SdCmd::AllSendCid)? {
            SdCmdResponse::R2(cid) => cid,
            _ => panic!("Incorrect response type!"),
        };

        self.cid = SdCid::new(cid);

        // Get the RCA
        let rca = match self.send_command(SdCmd::SendRelativeAddr)? {
            SdCmdResponse::R6(rca) => rca,
            _ => {
                panic!("Incorrect response type!");
            }
        };

        self.rca = rca;

        // Get the CSD
        let csd = match self.send_command(SdCmd::SendCsd(rca))? {
            SdCmdResponse::R2(csd) => csd,
            _ => {
                panic!("Incorrect response type!");
            }
        };

        self.csd = SdCsd::new(csd);

        // Select the card
        self.send_command(SdCmd::SelectDeselectCard(rca))?;

        // Set the bus width to 4 bits(true means 4 bits)
        self.send_command(SdCmd::SetBusWidth(true))?;

        // Set the block lenth
        self.send_command(SdCmd::SetBlockLen(SD_BLOCK_LEN as u32))?;

        // Speed up the clock to 25mhz
        self.sm_cmd
            .clock_divisor_fixed_point(self.sd_full_clk_div, 0);

        // Read the first block
        self.start_block_rx()?;
        self.wait_rx()?;
        Ok(())
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt> Io for Sdio4bit<'a, DmaCh, P> {
    type Error = SdioError;
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt> Read for Sdio4bit<'a, DmaCh, P> {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, SdioError> {
        // Wait until we are doing nothing
        self.wait_rx_tx()?;

        // If the position isn't in the current working block, flush,
        // then load the correct working block
        let current_block_num = (self.position / (SD_BLOCK_LEN as u64)) as u32;

        if current_block_num != self.working_block_num {
            self.flush()?;

            self.working_block_num = current_block_num;
            self.start_block_rx()?;
            self.wait_rx()?;
        }

        let working_block = self.working_block.as_ref().unwrap();

        // Ensure buffer_index and self.position are incremented at
        // the same time
        let mut buffer_index: usize = 0;
        let mut block_index: usize =
            ((self.position % (SD_BLOCK_LEN as u64)) as usize) / SD_WORD_DIV;

        // First read all the bytes that don't align with words
        let offset = (self.position % (SD_WORD_DIV as u64)) as usize;
        let shift = offset * 8;

        // If the bytes aren't aligned
        if shift != 0 {
            let mut buffer_word: u32 = working_block[block_index] << shift;

            while (self.position % (SD_WORD_DIV as u64)) != 0 && buffer.len() > buffer_index {
                buffer[buffer_index] = (buffer_word >> 24) as u8;
                buffer_word <<= 8;

                buffer_index += 1;
                self.position += 1;
            }

            block_index += 1;
        }

        // Now that the bytes are aligned, time to convert them all
        // (until we can't)
        while (buffer.len() - buffer_index) >= SD_WORD_DIV {
            // Check if the position is now outside of the working block,
            // and if so return with bytes written
            if block_index >= (SD_BLOCK_LEN / SD_WORD_DIV) {
                return Ok(buffer_index);
            }

            let buffer_bytes = working_block[block_index].to_be_bytes();

            buffer[buffer_index..buffer_index + 4].copy_from_slice(&buffer_bytes);

            // Increment the indicies
            buffer_index += SD_WORD_DIV as usize;
            self.position += SD_WORD_DIV as u64;
            block_index += 1;
        }

        // Lastly write all the bytes that don't align with words(again)
        let remaining = buffer.len() - buffer_index;

        // If the bytes aren't aligned...
        if remaining > 0 {
            // Check if the position is now outside of the working block,
            // and if so return with bytes written
            if block_index >= (SD_BLOCK_LEN / SD_WORD_DIV) {
                return Ok(buffer_index);
            }

            let mut buffer_word: u32 = working_block[block_index];

            while buffer.len() > buffer_index {
                buffer[buffer_index] = (buffer_word >> 24) as u8;

                buffer_word <<= 8;

                buffer_index += 1;
                self.position += 1;
            }
        }

        Ok(buffer_index)
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt> Write for Sdio4bit<'a, DmaCh, P> {
    /// Write bytes to the sd card. The PIO requires bytes to be converted to
    /// little endian words, so there is a lot of conversion that needs to take
    /// place...
    /// Will return when the end of the block is encountered so be mindful...
    fn write(&mut self, buffer: &[u8]) -> Result<usize, SdioError> {
        // Wait until we are doing nothing
        self.wait_rx_tx()?;

        // If the position isn't in the current working block, flush,
        // then load the correct working block
        let current_block_num = (self.position / (SD_BLOCK_LEN as u64)) as u32;

        if current_block_num != self.working_block_num {
            self.flush()?;

            self.working_block_num = current_block_num;
            self.start_block_rx()?;
            self.wait_rx()?;
        }

        let working_block = &mut self.working_block.as_mut().unwrap();

        // Ensure buffer_index and self.position are incremented at
        // the same time
        let mut buffer_index: usize = 0;
        let mut block_index: usize =
            ((self.position % (SD_BLOCK_LEN as u64)) as usize) / SD_WORD_DIV;

        // First write all the bytes that don't align with words
        let offset = (self.position % (SD_WORD_DIV as u64)) as usize;
        let mut shift = 24 - (8 * offset) as usize;
        let remaining = cmp::min(buffer.len() + offset, 4);

        // If the bytes aren't aligned
        if shift != 24 {
            // Prepare a buffer word identical to the working block
            // Mask A keeps all bytes before the data intact, mask b keeps
            // all bytes after the data intact(if the length is 3 or under)
            let mask_a = 0xFFFF_FFFF << (shift + 8);
            // If we shift 32 it will overflow, so we have to shift twice...
            let mask_b = 0xFFFF_FFFF >> ((8 * remaining) - 1) >> 1;
            let mut buffer_word: u32 = working_block[block_index] & (mask_a | mask_b);

            while shift > 0 && buffer.len() > buffer_index {
                buffer_word |= (buffer[buffer_index] as u32) << shift;

                shift -= 8;
                buffer_index += 1;
                self.position += 1;
            }

            if buffer.len() > buffer_index {
                buffer_word |= buffer[buffer_index] as u32;
                buffer_index += 1;
                self.position += 1;
            }

            // Write the buffer word to the working block
            working_block[block_index] = buffer_word;
            block_index += 1;
        }

        // Now that the bytes are aligned, time to convert them all into words
        // (until we can't)
        while (buffer.len() - buffer_index) >= SD_WORD_DIV {
            // Check if the position is now outside of the working block,
            // and if so return with bytes written
            if block_index >= (SD_BLOCK_LEN / SD_WORD_DIV) {
                return Ok(buffer_index);
            }

            let buffer_slice = &buffer[buffer_index..buffer_index + SD_WORD_DIV];
            let buffer_word = u32::from_be_bytes(buffer_slice.try_into().unwrap());

            // Write the buffer word to the working block
            working_block[block_index] = buffer_word;

            // Increment the indicies
            buffer_index += SD_WORD_DIV as usize;
            self.position += SD_WORD_DIV as u64;
            block_index += 1;
        }

        // Lastly write all the bytes that don't align with words(again)
        let remaining = buffer.len() - buffer_index;

        // If the bytes aren't aligned...
        if remaining > 0 {
            // Check if the position is now outside of the working block,
            // and if so return with bytes written
            if block_index >= (SD_BLOCK_LEN / SD_WORD_DIV) {
                return Ok(buffer_index);
            }

            let mut shift: usize = 24;

            // Prepare a buffer word
            // The mask preserves all bytes after the data to be written
            let mask: u32 = 0xFFFF_FFFF >> (8 * remaining);
            let mut buffer_word: u32 = working_block[block_index] & mask;

            while buffer_index < buffer.len() {
                buffer_word |= (buffer[buffer_index] as u32) << shift;

                shift -= 8;
                buffer_index += 1;
                self.position += 1;
            }

            // Write the buffer word to the working block
            working_block[block_index] = buffer_word;
        }

        Ok(buffer_index)
    }

    fn flush(&mut self) -> Result<(), SdioError> {
        // If already in tx, we don't need to flush
        if self.is_tx() {
            self.wait_tx()?;
            return Ok(());
        }

        // Otherwise wait for the recieve to be finished
        self.wait_rx()?;

        self.start_block_tx()?;
        self.wait_tx()?;

        Ok(())
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt> Seek for Sdio4bit<'a, DmaCh, P> {
    fn seek(&mut self, position: SeekFrom) -> Result<u64, SdioError> {
        Ok(match position {
            SeekFrom::Start(position) => {
                self.position = position;
                self.position
            }
            SeekFrom::End(position) => {
                todo!()
            }
            SeekFrom::Current(delta) => {
                self.position = ((self.position as i64) + delta) as u64;
                self.position
            }
        })
    }
}
