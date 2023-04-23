//! Crate used to interface with SD cards via SDIO
//!
//! *currently in the alpha phase: works but missing important features and
//! is poorly optimised. future updates guarentee breaking changes.*

#![no_std]

pub mod errors;
pub mod registers;
pub mod sdio;
