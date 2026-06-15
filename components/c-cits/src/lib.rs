#![no_std]
#![feature(core_float_math)]

extern crate alloc;

mod conversions;
/// cbindgen:ignore
mod esp_alloc;
/// cbindgen:ignore
mod esp_panic;
/// cbindgen:ignore
mod esp_stdout;


pub mod gnss;
pub mod gnw;
pub mod its;
