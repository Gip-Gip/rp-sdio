
pub struct SdCid {
    cid: [u32; 4],
}

impl SdCid {
    pub fn new(cid: [u32; 4]) -> Self {
        Self{
            cid
        }
    }
}

pub struct SdCsd {
    csd: [u32; 4],
}

impl SdCsd {
    pub fn new(csd: [u32; 4]) -> Self {
        Self{
            csd
        }
    }
}

pub struct SdOcr {
    pub ocr: u32,
}

impl SdOcr {
    /// Returns the voltage window
    pub fn get_voltage_window(&self) -> u32 {
        self.ocr & 0xFF_FFFF
    }
    /// Returns true if the card is busy
    pub fn is_busy(&self) -> bool {
        (self.ocr & (1 << 31)) == 0
    }
    /// Returns true if the card supports switching to 1.8v
    pub fn s18a(&self) -> bool {
        (self.ocr & (1 << 24)) > 0
    }
    /// Returns true if the card supports a capacity over 2 terabytes
    pub fn co2t(&self) -> bool {
        (self.ocr & (1 << 27)) > 0
    }
    /// Returns true if the card supports UHS-II
    pub fn uhs2(&self) -> bool {
        (self.ocr & (1 << 29)) > 0
    }
    /// Returns true if the CCS bit is set
    pub fn ccs(&self) -> bool {
        (self.ocr & (1 << 30)) > 0
    }
}

pub struct SdCic {
    /// Set to true if the card
    pub supports_1p2v: bool,
    /// Set to true if the card supports pcie
    pub supports_pcie: bool,
}
