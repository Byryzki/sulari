use crate::Command;
use serde::{Deserialize, Serialize};

/// Response returned by the device upon completing the processing of a [Command]
///
/// Do not change the ABI (i.e., anything about the data structure), as the staff test code uses
/// exactly this binary layout to test the device.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[repr(C)]
pub enum Response {
    /// The message was accepted and processed as received. `Payload` is used for a command-specific
    /// return value.
    Ok(Option<Payload>),

    /// The message was not processed. [RejectReason] contains the reason for why the message was not processed.
    Rejected(RejectReason),

    /// The frame was corrupted, but a message was recovered and processed
    OkRecovered(Option<Payload>, Command),
}

/// Payload is used for a command-specific return value
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[repr(C)]
pub enum Payload {
    /// The internal counter value
    Counter(u64),
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[repr(C)]
/// The reason for why the input was rejected
pub enum RejectReason {
    /// The received frame was found to be invalid
    CorruptedFrame,
    /// The command was part of an illegal sequence of commands, or the command was found to be
    /// malformed in some other way.
    IllegalCommand,
    /// The command is not implemented by the device
    NotImplemented,
    /// There was an unspecified internal error during command processing
    ///
    /// Avoid producing this message when you can and instead produce [Response::Rejected(IllegalCommand)] if you
    /// think the serial input was invalid.
    ///
    /// The device may provide its own diagnostics over another serial channel (e.g., the RTT).
    InternalError,
}

impl Response {
    /// Whether the response is a positive one, as opposed to an error
    pub fn is_ok(&self) -> bool {
        match self {
            Response::Ok(_) | Response::OkRecovered(..) => true,
            Response::Rejected(_) => false,
        }
    }

    /// Returns the payload for `Ok` type responses or `None` for `Rejected` response
    pub fn payload(&self) -> Option<&Payload> {
        match self {
            Response::Ok(payload) | Response::OkRecovered(payload, _) => payload.as_ref(),
            Response::Rejected(_) => None,
        }
    }
}
