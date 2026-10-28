#include "hr/udp/hr_udp_link.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace hr::udp {

namespace {

constexpr uint8_t MAGIC_HELLO[4] = {'H', 'R', 'U', 'H'};
constexpr uint8_t MAGIC_CMD[4]   = {'H', 'R', 'U', 'C'};
constexpr uint8_t MAGIC_SVC[4]   = {'H', 'R', 'U', 'S'};
constexpr uint8_t MAGIC_ACK[4]   = {'H', 'R', 'U', 'A'};
constexpr uint8_t MAGIC_STATE[4] = {'H', 'R', 'U', 'T'};

constexpr std::size_t HELLO_SIZE   = 10u;   // <4sHI
constexpr std::size_t CMD_HDR_SIZE = 12u;   // <4sHIBB
constexpr std::size_t CMD_ENTRY_SIZE = 5u;  // <Bf
constexpr std::size_t SVC_SIZE     = 18u;   // <4sHIHBBf
constexpr std::size_t ACK_HDR_SIZE = 11u;   // <4sHIB
constexpr std::size_t ACK_MSG_MAX  = 900u;
constexpr std::size_t RX_BUF_SIZE  = 2048u;

inline void put_u16(uint8_t* p, uint16_t v) noexcept { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
inline void put_u32(uint8_t* p, uint32_t v) noexcept { for (int i = 0; i < 4; ++i) { p[i] = static_cast<uint8_t>(v >> (8 * i)); } }
inline void put_u64(uint8_t* p, uint64_t v) noexcept { for (int i = 0; i < 8; ++i) { p[i] = static_cast<uint8_t>(v >> (8 * i)); } }
inline void put_f32(uint8_t* p, float v) noexcept { uint32_t u; std::memcpy(&u, &v, 4); put_u32(p, u); }
inline uint16_t get_u16(const uint8_t* p) noexcept { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) noexcept
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline float get_f32(const uint8_t* p) noexcept { const uint32_t u = get_u32(p); float f; std::memcpy(&f, &u, 4); return f; }
inline bool magic_is(const uint8_t* p, const uint8_t (&m)[4]) noexcept { return std::memcmp(p, m, 4) == 0; }

bool same_addr(const sockaddr_in& a, const sockaddr_in& b) noexcept
{
    return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

std::string addr_text(const sockaddr_in& a)
{
    char buf[INET_ADDRSTRLEN] = {};
    if (::inet_ntop(AF_INET, &a.sin_addr, buf, sizeof(buf)) == nullptr) { return "?"; }
    return std::string(buf) + ":" + std::to_string(ntohs(a.sin_port));
}

} // namespace

// ---------------------------------------------------------------------------
std::size_t encode_state(const StateFrame& frame, uint32_t seq, uint8_t* out, std::size_t out_size) noexcept
{
    if (out_size < STATE_PACKET_SIZE) { return 0u; }
    uint8_t* p = out;
    std::memcpy(p, MAGIC_STATE, 4); p += 4;
    put_u16(p, PROTO_VERSION); p += 2;
    put_u32(p, seq); p += 4;
    put_u64(p, frame.timestamp_us); p += 8;
    *p++ = frame.fsm;
    *p++ = frame.run_mode;
    *p++ = frame.axes_connected;
    *p++ = frame.axes_enabled;
    *p++ = frame.axes_faulted;
    *p++ = frame.can_ok ? 1u : 0u;
    for (const AxisStatus& a : frame.axes) {
        *p++ = a.motor_id;
        *p++ = a.connected ? 1u : 0u;
        *p++ = a.axis_state;
        *p++ = a.has_fault ? 1u : 0u;
        *p++ = a.channel;
        *p++ = 0u; *p++ = 0u; *p++ = 0u;          // pad[3]
        put_f32(p, a.position); p += 4;
        put_f32(p, a.velocity); p += 4;
        put_f32(p, a.effort);   p += 4;
    }
    return static_cast<std::size_t>(p - out);
}

std::size_t encode_ack(uint32_t seq, bool ok, const std::string& message, uint8_t* out, std::size_t out_size) noexcept
{
    const std::size_t msg_len = message.size() < ACK_MSG_MAX ? message.size() : ACK_MSG_MAX;
    if (out_size < ACK_HDR_SIZE + msg_len) { return 0u; }
    uint8_t* p = out;
    std::memcpy(p, MAGIC_ACK, 4); p += 4;
    put_u16(p, PROTO_VERSION); p += 2;
    put_u32(p, seq); p += 4;
    *p++ = ok ? 1u : 0u;
    std::memcpy(p, message.data(), msg_len); p += msg_len;
    return static_cast<std::size_t>(p - out);
}

// ---------------------------------------------------------------------------
Link::Link(LinkConfig cfg) : cfg_(std::move(cfg)) {}

Link::~Link() { close(); }

bool Link::open()
{
    close();
    if (!cfg_.enabled) { return false; }

    fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) {
        std::perror("[HR-UDP] socket");
        return false;
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(cfg_.listen_port);
    if (cfg_.bind_addr.empty() || cfg_.bind_addr == "0.0.0.0") {
        local.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, cfg_.bind_addr.c_str(), &local.sin_addr) != 1) {
        std::fprintf(stderr, "[HR-UDP] invalid bind_addr '%s'\n", cfg_.bind_addr.c_str());
        close();
        return false;
    }
    if (::bind(fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        std::fprintf(stderr, "[HR-UDP] bind %s:%u failed: %s (is hr_udp_bridge.py still running?)\n",
                     cfg_.bind_addr.c_str(), static_cast<unsigned>(cfg_.listen_port), std::strerror(errno));
        close();
        return false;
    }

    peer_valid_ = false;
    peer_fixed_ = false;
    if (!cfg_.peer_host.empty()) {
        sockaddr_in p{};
        p.sin_family = AF_INET;
        p.sin_port = htons(cfg_.peer_port);
        if (::inet_pton(AF_INET, cfg_.peer_host.c_str(), &p.sin_addr) == 1) {
            peer_ = p;
            peer_valid_ = true;
            peer_fixed_ = true;
        } else {
            std::fprintf(stderr, "[HR-UDP] invalid peer_host '%s' — will learn the peer from HELLO\n", cfg_.peer_host.c_str());
        }
    }
    return true;
}

void Link::close() noexcept
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

uint32_t Link::state_period_us() const noexcept
{
    if (cfg_.state_rate_hz == 0u) { return 0u; }
    return 1000000u / cfg_.state_rate_hz;
}

bool Link::has_peer(uint64_t now_us) const noexcept
{
    if (!peer_valid_) { return false; }
    if (peer_fixed_ || cfg_.peer_timeout_ms == 0u) { return true; }
    const uint64_t timeout_us = static_cast<uint64_t>(cfg_.peer_timeout_ms) * 1000u;
    return now_us <= peer_last_rx_us_ + timeout_us;
}

std::string Link::peer_text() const
{
    if (!peer_valid_) { return "(none)"; }
    return addr_text(peer_) + (peer_fixed_ ? " (fixed)" : "");
}

LinkStats Link::take_stats() noexcept
{
    const LinkStats s = stats_;
    stats_ = LinkStats{};
    return s;
}

void Link::learn_peer(const sockaddr_in& from, uint64_t now_us, bool from_hello)
{
    peer_last_rx_us_ = now_us;
    if (peer_fixed_) { return; }
    if (!peer_valid_ || (from_hello && !same_addr(peer_, from))) {
        peer_ = from;
        peer_valid_ = true;
        std::printf("[HR-UDP] peer %s registered (%s)\n", addr_text(from).c_str(), from_hello ? "HELLO" : "first packet");
    }
}

bool Link::send_raw(const uint8_t* data, std::size_t len, const sockaddr_in& to) noexcept
{
    if (fd_ < 0) { return false; }
    const ssize_t n = ::sendto(fd_, data, len, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&to), sizeof(to));
    if (n != static_cast<ssize_t>(len)) {
        ++stats_.tx_fail;
        return false;
    }
    return true;
}

bool Link::send_state(const StateFrame& frame, uint64_t now_us)
{
    if (!has_peer(now_us)) { return false; }
    uint8_t buf[STATE_PACKET_SIZE];
    const std::size_t n = encode_state(frame, ++state_seq_, buf, sizeof(buf));
    if (n == 0u) { return false; }
    if (!send_raw(buf, n, peer_)) { return false; }
    ++stats_.tx_state;
    return true;
}

void Link::drain(const CmdHandler& on_cmd, const SvcHandler& on_svc, uint64_t now_us)
{
    if (fd_ < 0) { return; }
    uint8_t buf[RX_BUF_SIZE];
    for (int guard = 0; guard < 256; ++guard) {           // bounded: never starve the caller
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), MSG_DONTWAIT,
                                     reinterpret_cast<sockaddr*>(&from), &from_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) { return; }
            ++stats_.rx_bad;
            return;
        }
        if (from.sin_family != AF_INET) { ++stats_.rx_bad; continue; }
        handle_packet(buf, static_cast<std::size_t>(n), from, on_cmd, on_svc, now_us);
    }
}

void Link::handle_packet(const uint8_t* data, std::size_t len, const sockaddr_in& from,
                         const CmdHandler& on_cmd, const SvcHandler& on_svc, uint64_t now_us)
{
    if (len < 6u || get_u16(data + 4) != PROTO_VERSION) { ++stats_.rx_bad; return; }

    if (magic_is(data, MAGIC_HELLO)) {
        if (len < HELLO_SIZE) { ++stats_.rx_bad; return; }
        ++stats_.rx_hello;
        learn_peer(from, now_us, true);
        return;
    }

    if (magic_is(data, MAGIC_CMD)) {
        if (len < CMD_HDR_SIZE) { ++stats_.rx_bad; return; }
        const uint8_t mode = data[10];
        const std::size_t count = data[11];
        if (len < CMD_HDR_SIZE + count * CMD_ENTRY_SIZE) { ++stats_.rx_bad; return; }
        ++stats_.rx_cmd;
        stats_.rx_cmd_entries += count;
        learn_peer(from, now_us, false);
        CommandEntry entries[MAX_CMD_ENTRIES];
        for (std::size_t k = 0; k < count; ++k) {
            const uint8_t* e = data + CMD_HDR_SIZE + k * CMD_ENTRY_SIZE;
            entries[k].motor_id = e[0];
            entries[k].position = get_f32(e + 1);
        }
        if (on_cmd) { on_cmd(mode, entries, count); }
        return;
    }

    if (magic_is(data, MAGIC_SVC)) {
        if (len < SVC_SIZE) { ++stats_.rx_bad; return; }
        ++stats_.rx_svc;
        learn_peer(from, now_us, false);
        const uint32_t seq = get_u32(data + 6);
        const uint16_t svc = get_u16(data + 10);
        const uint8_t  motor_id = data[12];
        const float    value = get_f32(data + 14);
        SvcResult result{false, "no handler"};
        if (on_svc) { result = on_svc(svc, motor_id, value); }
        uint8_t ack[ACK_HDR_SIZE + ACK_MSG_MAX];
        const std::size_t n = encode_ack(seq, result.ok, result.message, ack, sizeof(ack));
        if (n > 0u && send_raw(ack, n, from)) { ++stats_.tx_ack; }
        return;
    }

    ++stats_.rx_bad;   // ACK / STATE are never expected on this side
}

} // namespace hr::udp
