//! This crate defines the binary layout of the messages in the-protocol
//!
//! N.b., the staff uses these same data structures and their binary layout to perform the testing
//! of your solution. **Do not change anything in this library.**
#![no_std]
// The ABI must be documented thoroughly
#![deny(missing_docs)]

mod cmd;
mod date_time;
mod response;

pub use cmd::{Command, Funct};
pub use date_time::SDateTime;
pub use response::{Payload, RejectReason, Response};

// Expose the exact version of libraries that are used as part of the API (and the ABI)
pub use chrono;
