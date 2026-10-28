#pragma once
// ---------------------------------------------------------------------------
// hr::udp::Link — direct UDP link between hr_controller and the Jetson
// (quest_arm_ik `hr_udp_relay`).  Replaces the Python ros2_state_bridge /
// hr_udp_bridge path for the teleoperation host: no JSON file, no IPC hop,
// no DDS on the cable.
//
// Wire protocol (little-endian, every packet fits one Ethernet frame):
//   HELLO  host → ctrl  'HRUH' ver u16, seq u32                          (peer registration, ~1 Hz)
//   CMD    host → ctrl  'HRUC' ver u16, seq u32, mode u8, n u8, n×(motor_id u8, position f32)
//   SVC    host → ctrl  'HRUS' ver u16, seq u32, svc u16, motor_id u8, pad u8, value f32
//   ACK    ctrl → host  'HRUA' ver u16, seq u32, ok u8, message (utf-8)
//   STATE  ctrl → host  'HRUT' ver u16, seq u32, timestamp_us u64, fsm u8, run_mode u8,
//                       axes_connected u8, axes_enabled u8, axes_faulted u8, can_ok u8,
//                       15×(motor_id u8, connected u8, axis_state u8, has_fault u8, channel u8,
//                           pad[3], position f32, velocity f32, effort f32)             = 324 B
// svc numbers equal IpcCommandType (1 ENABLE_ALL, 2 DISABLE_ALL, 3 EMERGENCY_STOP, 4 RESET_FAULT,
// 6 ENABLE_AXIS, 7 DISABLE_AXIS, 9 SET_RUN_MODE (value = mode), 12 HOME_AXIS, 13 RESCAN_UID).
// fsm uses the ros2_interfaces/UpperBodySystemState numbering (0 POWER_OFF, 1 DRIVER_INIT,
// 2 READY, 3 ENABLING, 4 ENABLED, 5 RUNNING, 6 FAULT).
//
// This module depends only on POSIX sockets and the C++ standard library so it can be
// unit-tested without the iRobe SDK.  All socket operations are non-blocking.
// ---------------------------------------------------------------------------
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include <netinet/in.h>

namespace hr::udp {

inline constexpr uint16_t    PROTO_VERSION = 1u;
inline constexpr std::size_t LINK_AXES     = 15u;
inline constexpr std::size_t STATE_PACKET_SIZE = 24u + LINK_AXES * 20u;   // 324
inline constexpr std::size_t MAX_CMD_ENTRIES   = 255u;

// svc numbers (== IpcCommandType in main.cpp)
inline constexpr uint16_t SVC_ENABLE_ALL     = 1u;
inline constexpr uint16_t SVC_DISABLE_ALL    = 2u;
inline constexpr uint16_t SVC_EMERGENCY_STOP = 3u;
inline constexpr uint16_t SVC_RESET_FAULT    = 4u;
inline constexpr uint16_t SVC_ENABLE_AXIS    = 6u;
inline constexpr uint16_t SVC_DISABLE_AXIS   = 7u;
inline constexpr uint16_t SVC_SET_RUN_MODE   = 9u;
inline constexpr uint16_t SVC_HOME_AXIS      = 12u;
inline constexpr uint16_t SVC_RESCAN_UID     = 13u;

// fsm numbers on the wire (UpperBodySystemState)
inline constexpr uint8_t FSM_POWER_OFF   = 0u;
inline constexpr uint8_t FSM_DRIVER_INIT = 1u;
inline constexpr uint8_t FSM_READY       = 2u;
inline constexpr uint8_t FSM_ENABLING    = 3u;
inline constexpr uint8_t FSM_ENABLED     = 4u;
inline constexpr uint8_t FSM_RUNNING     = 5u;
inline constexpr uint8_t FSM_FAULT       = 6u;

struct LinkConfig {
    bool        enabled{true};
    std::string bind_addr{"0.0.0.0"};
    uint16_t    listen_port{7001};
    std::string peer_host{};          // empty = learn the peer from HELLO / CMD / SVC packets
    uint16_t    peer_port{7002};
    uint32_t    state_rate_hz{100};   // STATE packets per second (0 = never)
    uint32_t    peer_timeout_ms{3000}; // learned peer is forgotten after this silence (0 = never)
};

struct AxisStatus {
    uint8_t motor_id{255};
    bool    connected{false};
    uint8_t axis_state{255};   // hr::AxisState (2 = Run); 255 = unknown / no axis
    bool    has_fault{false};
    uint8_t channel{0};
    float   position{0.0f};
    float   velocity{0.0f};
    float   effort{0.0f};
};

struct StateFrame {
    uint64_t timestamp_us{0};
    uint8_t  fsm{FSM_POWER_OFF};
    uint8_t  run_mode{0};
    uint8_t  axes_connected{0};
    uint8_t  axes_enabled{0};
    uint8_t  axes_faulted{0};
    bool     can_ok{false};
    std::array<AxisStatus, LINK_AXES> axes{};
};

struct CommandEntry {
    uint8_t motor_id{0};
    float   position{0.0f};
};

struct SvcResult {
    bool        ok{false};
    std::string message{};
};

struct LinkStats {
    uint64_t rx_hello{0};
    uint64_t rx_cmd{0};
    uint64_t rx_cmd_entries{0};
    uint64_t rx_svc{0};
    uint64_t rx_bad{0};
    uint64_t tx_state{0};
    uint64_t tx_ack{0};
    uint64_t tx_fail{0};
};

// Called for every CMD packet: mode (UpperBodyCommand.command_mode) and the entries.
using CmdHandler = std::function<void(uint8_t mode, const CommandEntry* entries, std::size_t count)>;
// Called for every SVC packet; the result is sent back as ACK.
using SvcHandler = std::function<SvcResult(uint16_t svc, uint8_t motor_id, float value)>;

// Pure packet (de)serialisation — exposed for tests.
std::size_t encode_state(const StateFrame& frame, uint32_t seq, uint8_t* out, std::size_t out_size) noexcept;
std::size_t encode_ack(uint32_t seq, bool ok, const std::string& message, uint8_t* out, std::size_t out_size) noexcept;

class Link {
public:
    explicit Link(LinkConfig cfg);
    ~Link();
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    // Create + bind the socket.  Returns false (and prints to stderr) on failure.
    bool open();
    void close() noexcept;
    [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int  fd() const noexcept { return fd_; }
    [[nodiscard]] const LinkConfig& config() const noexcept { return cfg_; }

    // Drain every pending datagram (never blocks).  Handlers run inline.
    void drain(const CmdHandler& on_cmd, const SvcHandler& on_svc, uint64_t now_us);

    // Send one STATE packet to the peer, if any.  Returns true when a datagram went out.
    bool send_state(const StateFrame& frame, uint64_t now_us);

    [[nodiscard]] bool        has_peer(uint64_t now_us) const noexcept;
    [[nodiscard]] std::string peer_text() const;
    [[nodiscard]] uint32_t    state_period_us() const noexcept;

    // Returns the counters accumulated since the previous call and resets them.
    LinkStats take_stats() noexcept;

private:
    void handle_packet(const uint8_t* data, std::size_t len, const sockaddr_in& from,
                       const CmdHandler& on_cmd, const SvcHandler& on_svc, uint64_t now_us);
    void learn_peer(const sockaddr_in& from, uint64_t now_us, bool from_hello);
    bool send_raw(const uint8_t* data, std::size_t len, const sockaddr_in& to) noexcept;

    LinkConfig  cfg_;
    int         fd_{-1};
    sockaddr_in peer_{};
    bool        peer_valid_{false};
    bool        peer_fixed_{false};
    uint64_t    peer_last_rx_us_{0};
    uint32_t    state_seq_{0};
    LinkStats   stats_{};
};

} // namespace hr::udp
