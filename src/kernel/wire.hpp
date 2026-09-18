// Minimal hand-rolled protobuf wire encoder for the enforcement event
// messages. Exists only because build brief §6 constraint 6 forbids heap
// allocation on hk_gate's path: the generated protobuf C++ API allocates
// (Arena aside, still not a fit for a caller-owned flat buffer with no
// pre-known size), so the event messages — small, fixed field numbers, never
// changing shape without a schema bump — are encoded by hand instead.
//
// Two writer kinds share one set of encode_* functions (wire.hpp callers
// write generic code against the `W` template parameter):
//   SizeWriter — no bytes written, just accumulates the encoded size. Used to
//                compute the varint length prefix of a submessage before its
//                bytes are known.
//   BufWriter  — writes into a caller-owned buffer with bounds checking; sets
//                `overflow` rather than throwing or growing.
// A field is written by first running a full SizeWriter pass over it, then
// letting the real writer (Size or Buf) append: for a SizeWriter this just
// keeps accumulating (arriving at the same total as if it were the sole
// pass); for a BufWriter it emits the actual bytes. No heap allocation
// happens in either path — SizeWriter and BufWriter are both stack values.

#ifndef HARNESS_KERNEL_WIRE_HPP
#define HARNESS_KERNEL_WIRE_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace harness::wire {

enum class WireType : std::uint32_t { Varint = 0, Fixed64 = 1, Len = 2, Fixed32 = 5 };

constexpr std::size_t varint_size(std::uint64_t v) {
    std::size_t n = 1;
    while (v >= 0x80U) {
        v >>= 7U;
        ++n;
    }
    return n;
}

struct SizeWriter {
    std::size_t size = 0;

    void tag(std::uint32_t field, WireType wt) {
        size += varint_size((static_cast<std::uint64_t>(field) << 3U) |
                             static_cast<std::uint64_t>(wt));
    }
    void varint(std::uint64_t v) { size += varint_size(v); }
    void fixed64(std::uint64_t) { size += 8; }
    void bytes(const void*, std::size_t n) { size += n; }
};

struct BufWriter {
    std::uint8_t* buf = nullptr;
    std::size_t cap = 0;
    std::size_t pos = 0;
    bool overflow = false;

    void raw(const void* p, std::size_t n) {
        if (overflow || pos + n > cap) {
            overflow = true;
            return;
        }
        std::memcpy(buf + pos, p, n);
        pos += n;
    }
    void tag(std::uint32_t field, WireType wt) {
        varint((static_cast<std::uint64_t>(field) << 3U) | static_cast<std::uint64_t>(wt));
    }
    void varint(std::uint64_t v) {
        std::uint8_t tmp[10];
        std::size_t n = 0;
        while (v >= 0x80U) {
            tmp[n++] = static_cast<std::uint8_t>(v) | 0x80U;
            v >>= 7U;
        }
        tmp[n++] = static_cast<std::uint8_t>(v);
        raw(tmp, n);
    }
    void fixed64(std::uint64_t bits) { raw(&bits, 8); }
    void bytes(const void* p, std::size_t n) { raw(p, n); }
};

// Scalar field helpers. proto3 semantics: omitting a default-valued field is
// equivalent on the wire to emitting it, so for simplicity every helper here
// always emits (event writers always set every field they emit, per build
// brief §3 — the omit-if-default optimisation buys nothing for this schema).

template <typename W>
void put_uint64(W& w, std::uint32_t field, std::uint64_t v) {
    w.tag(field, WireType::Varint);
    w.varint(v);
}

template <typename W>
void put_uint32(W& w, std::uint32_t field, std::uint32_t v) {
    put_uint64(w, field, v);
}

template <typename W>
void put_enum(W& w, std::uint32_t field, int v) {
    put_uint64(w, field, static_cast<std::uint64_t>(static_cast<std::int64_t>(v)));
}

template <typename W>
void put_bool(W& w, std::uint32_t field, bool v) {
    put_uint64(w, field, v ? 1U : 0U);
}

inline std::uint64_t double_bits(double d) {
    std::uint64_t bits;
    std::memcpy(&bits, &d, 8);
    return bits;
}

template <typename W>
void put_double(W& w, std::uint32_t field, double v) {
    w.tag(field, WireType::Fixed64);
    w.fixed64(double_bits(v));
}

template <typename W>
void put_string(W& w, std::uint32_t field, std::string_view s) {
    w.tag(field, WireType::Len);
    w.varint(s.size());
    w.bytes(s.data(), s.size());
}

template <typename W>
void put_bytes(W& w, std::uint32_t field, const void* data, std::size_t n) {
    w.tag(field, WireType::Len);
    w.varint(n);
    w.bytes(data, n);
}

template <typename W>
void put_packed_double(W& w, std::uint32_t field, const double* values, std::size_t n) {
    if (n == 0) return;
    w.tag(field, WireType::Len);
    w.varint(n * 8U);
    for (std::size_t i = 0; i < n; ++i) {
        w.fixed64(double_bits(values[i]));
    }
}

template <typename W>
void put_packed_enum(W& w, std::uint32_t field, const std::int32_t* values, std::size_t n) {
    if (n == 0) return;
    SizeWriter inner;
    for (std::size_t i = 0; i < n; ++i) inner.varint(static_cast<std::uint64_t>(values[i]));
    w.tag(field, WireType::Len);
    w.varint(inner.size);
    for (std::size_t i = 0; i < n; ++i) w.varint(static_cast<std::uint64_t>(values[i]));
}

// Submessage embedding: run a throwaway SizeWriter pass to learn the encoded
// length, write the length-delimited tag+len on `w`, then let `encode`
// append the payload on `w` itself (a no-op accumulation for SizeWriter, the
// real bytes for BufWriter).
template <typename W, typename Msg, typename EncodeFn>
void put_message(W& w, std::uint32_t field, const Msg& m, EncodeFn&& encode) {
    SizeWriter inner;
    encode(inner, m);
    w.tag(field, WireType::Len);
    w.varint(inner.size);
    encode(w, m);
}

}  // namespace harness::wire

#endif  // HARNESS_KERNEL_WIRE_HPP
