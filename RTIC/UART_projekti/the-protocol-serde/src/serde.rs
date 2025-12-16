use crate::Codec;
use corncobs::max_encoded_len;

#[derive(Debug)]
pub enum SerializeError {
    // TODO: you may add more error variants here
}

#[derive(Debug)]
pub enum DeserializeError {
    // TODO: you may add more error variants here
}

// Blanket implementation of Codec for all types `P` that implement `serde::{Deserialize, Serialize}`
impl<P> Codec<P> for P
where
    // `P` must be both serializable and deserializable to form a valid payload
    P: for<'de> serde::Deserialize<'de> + serde::Serialize,
{
    type DeserializeError = DeserializeError;
    type SerializeError = SerializeError;

    /// Maximum serialized len. We will leave one word extra for implementations to play with (e.g.,
    /// if an implementation wants to include the framing zero on the decode side.)
    const MAX_SERIALIZED_LEN: usize = max_encoded_len(size_of::<P>() + size_of::<u32>());

    /// Serialize an instance of type `T` into a COBS packet. Returns the sub-slice of `out_buf`
    /// that was allocated.
    ///
    /// # Arguments
    ///
    /// * `value` - the value to serialize
    ///
    /// # Type arguments
    ///
    /// * `'a` - lifetime of `out_buf` which bounds also the lifetime of return value which is a view to `out_buf`
    /// * `N` - buffer size for the output packet, i.e., maximum size after serialization and anything
    ///   else you might want to include in the packet. This should generally match `MAX_SERIALIZED_LEN`.
    ///
    /// # Errors
    ///
    /// * panics on all errors
    ///
    /// TODO: can we do better?
    fn serialize<'a, const N: usize>(
        &self,
        out_buf: &'a mut [u8; N],
    ) -> Result<&'a mut [u8], Self::SerializeError> {
        // Serialize the value
        let n_ser = ssmarshal::serialize(out_buf, self).unwrap();
        // memcpy
        let buf_copy = *out_buf;
        // Encode the whole message into a COBS packet
        let n = corncobs::encode_buf(&buf_copy[0..n_ser], out_buf);
        Ok(&mut out_buf[0..n])
    }

    /// Deserialize an instance of type `T` from a COBS packet
    ///
    /// # Arguments
    ///
    /// * `in_buf` - the bytes of a COBS packet
    ///
    /// # Errors
    ///
    /// * panics on all errors
    ///
    /// TODO: can we do better?
    fn deserialize_in_place(in_buf: &mut [u8]) -> Result<Self, Self::DeserializeError> {
        let n = corncobs::decode_in_place(in_buf).unwrap();
        let (t, _bytes_used) = ssmarshal::deserialize(&in_buf[0..n]).unwrap();
        Ok(t)
    }
}
