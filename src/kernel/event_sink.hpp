// Writes one encoded EnforcementEvent (length-prefixed) into a caller-owned
// hk_event_sink. This is the only place that touches the sink's raw buffer;
// hk_gate and friends never poke at `buf`/`len`/`dropped` directly.

#ifndef HARNESS_KERNEL_EVENT_SINK_HPP
#define HARNESS_KERNEL_EVENT_SINK_HPP

#include "harness/harness_kernel.h"
#include "kernel/event_codec.hpp"
#include "kernel/wire.hpp"

namespace harness::kernel {

// Returns true if the event was written, false if the sink was full (in
// which case `*sink->dropped` has already been incremented). Never blocks,
// never throws, never touches the heap.
template <typename BodyFields, typename BodyEncodeFn>
bool try_write_event(hk_event_sink* sink, const EventHeader& header, std::uint32_t body_field,
                      const BodyFields& body, BodyEncodeFn&& body_encode) {
    wire::SizeWriter size_pass;
    encode_enforcement_event(size_pass, header, body_field, body, body_encode);

    std::size_t prefix_len = wire::varint_size(size_pass.size);
    std::size_t record_len = prefix_len + size_pass.size;

    if (sink == nullptr || sink->buf == nullptr || sink->len == nullptr ||
        sink->dropped == nullptr) {
        return false;
    }
    if (*sink->len + record_len > sink->cap) {
        (*sink->dropped)++;
        return false;
    }

    wire::BufWriter prefix_writer{sink->buf, sink->cap, *sink->len, false};
    prefix_writer.varint(size_pass.size);

    wire::BufWriter body_writer{sink->buf, sink->cap, prefix_writer.pos, false};
    encode_enforcement_event(body_writer, header, body_field, body, body_encode);

    if (body_writer.overflow) {
        // Unreachable given the size pre-pass above, but if it ever happens
        // it is a drop like any other and must be counted like one --
        // otherwise the caller's tally and the kernel's disagree silently.
        (*sink->dropped)++;
        return false;
    }
    *sink->len = body_writer.pos;
    return true;
}

}  // namespace harness::kernel

#endif  // HARNESS_KERNEL_EVENT_SINK_HPP
