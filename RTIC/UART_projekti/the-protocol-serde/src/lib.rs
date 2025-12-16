//! This crate defines the transport layer and serialization--deserialization API of messages
//! between the host & device applications.
//!
//! N.b., the staff testing software uses the methods defined by [Codec] to perform the testing of
//! your solution, so do you should consider whether you can alter something before you do.
//!
//! * For phase 1, you MAY change at least the error types returned by this API.
//! * For phase 2 you MAY change anything except the method signatures defined by the [Codec] trait.
#![no_std]
// The serialization layer must be documented thoroughly
#![deny(missing_docs)]

mod codec;
mod serde;

pub use codec::Codec;
pub use corncobs;

// Expose all visible items from [the_protocol]
pub use the_protocol::*;
