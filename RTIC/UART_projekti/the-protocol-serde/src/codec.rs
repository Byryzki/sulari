//! Stable interface for the-protocol for phases 1 & 2.
use core::fmt;

/// Trait for messages that can be passed to and from the device over a serial port
///
/// **The interface provided by this trait SHALL not be altered** as that will make the code
/// incompatible with test software in phase 2.
///
/// The existence of this trait allows to test phase 2 requirements even if the binary layout of the
/// protocol is changed. Users, e.g., automated testing software, may rely on the methods of this
/// API to stay consistent even while the binary layout of the protocol itself is changed. This
/// allows you to add reliability layers to the protocol.
///
/// # Type arguments
///
/// * `P` - Message payload type. Must be serializable and deserializable for transfer by serial
///   wire. Fulfilled by [crate::Command], [crate::Response] and any wrapped types.
pub trait Codec<P>
where
    Self: Sized,
    // `P` must be both serializable and deserializable to form a valid payload
    P: for<'de> serde::Deserialize<'de> + serde::Serialize,
{
    /// Error type representing what can go wrong during serialization. This type must implement
    ///   `Debug` so that it can be inspected by tests without knowledge of the actual type.
    type SerializeError: fmt::Debug;
    /// Error type representing what can go wrong during deserialization. This type must implement
    ///   `Debug` so that it can be inspected by tests without knowledge of the actual type.
    type DeserializeError: fmt::Debug;

    /// Maximum length of the encoded message in bytes
    const MAX_SERIALIZED_LEN: usize;

    /// Serialize an instance of type `P` into bytes for transfer by serial. Returns the sub-slice of
    /// `out_buf` that was allocated for the encoded packet.
    ///
    /// # Arguments
    ///
    /// * `self` - The input value to serialize
    /// * `out_buf` - The buffer to use for the encoded value
    ///
    /// # Type arguments
    ///
    /// * `N` - Buffer size for the output packet, i.e., maximum size after serialization and anything
    ///   else you might want to include in the packet.
    fn serialize<'a, const N: usize>(
        &self,
        out_buf: &'a mut [u8; N],
    ) -> Result<&'a mut [u8], Self::SerializeError>;

    /// Deserialize an instance of type `P` from bytes received by serial
    ///
    /// # Arguments
    ///
    /// * `in_buf` - The bytes of a COBS packet. This buffer will be reused for the deserialized
    ///   packet.
    fn deserialize_in_place(in_buf: &mut [u8]) -> Result<Self, Self::DeserializeError>;
}
