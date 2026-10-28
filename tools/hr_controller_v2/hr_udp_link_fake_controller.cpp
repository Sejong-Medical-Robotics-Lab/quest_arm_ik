// Fake hr_controller built on hr::udp::Link — exercises the exact code path main.cpp uses,
// without motors or the iRobe SDK.  Used by quest_arm_ik/test/test_udp_bridge.py --cpp.
//
//   g++ -std=c++20 -O2 -I<include dir> hr_udp_link_fake_controller.cpp hr_udp_link.cpp -o fake_ctrl
//   ./fake_ctrl --listen-port 17001 --state-rate 50 --duration 10
//
// Behaviour (mirrors test_udp_bridge.FakeController):
//   ENABLE_ALL  → every axis Run(2), fsm OD_CONTROL      DISABLE_ALL → Reset(0), READY
//   EMERGENCY_STOP → fsm EMERGENCY_STOP                  RESET_FAULT → READY
//   CMD (mode 1/6) → per enabled axis: target = position, first-order tracking in the 5 ms tick
//   SET_RUN_MODE value → recorded;  prints "moves=<n> svc=<n> run_mode_sets=<n>" on exit.
#include "hr/udp/hr_udp_link.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <string>

namespace {

std::atomic_bool g_running{true};
void on_signal(int) { g_running.store(false); }

constexpr std::array<uint8_t, 15> MOTORS = {34, 113, 125, 52, 105, 115, 57, 51, 32, 33, 31, 35, 36, 37, 255};

uint64_t now_us()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

struct Fake {
    std::string fsm{"READY"};
    std::array<uint8_t, 15> axis_state{};
    std::array<float, 15> pos{};
    std::array<float, 15> target{};
    uint64_t moves{0}, svcs{0}, run_mode_sets{0}, last_mode_value{0};

    Fake() { axis_state.fill(0); axis_state[14] = 255; }

    int axis_of(uint8_t motor_id) const
    {
        for (int i = 0; i < 14; ++i) { if (MOTORS[i] == motor_id) { return i + 1; } }
        return 0;
    }

    uint8_t fsm_code() const
    {
        if (fsm == "READY") return hr::udp::FSM_READY;
        if (fsm == "OD_CONTROL" || fsm == "IMPEDANCE_CONTROL") return hr::udp::FSM_ENABLED;
        if (fsm == "MOTION_RUNNING") return hr::udp::FSM_RUNNING;
        if (fsm == "FAULT" || fsm == "EMERGENCY_STOP") return hr::udp::FSM_FAULT;
        return hr::udp::FSM_POWER_OFF;
    }

    void on_cmd(uint8_t mode, const hr::udp::CommandEntry* e, std::size_t n)
    {
        if (mode != 1u && mode != 6u) { return; }
        for (std::size_t k = 0; k < n; ++k) {
            const int axis = axis_of(e[k].motor_id);
            if (axis == 0 || axis_state[axis - 1] != 2u) { continue; }
            target[axis - 1] = e[k].position;
            ++moves;
        }
    }

    hr::udp::SvcResult on_svc(uint16_t svc, uint8_t motor_id, float value)
    {
        ++svcs;
        switch (svc) {
        case hr::udp::SVC_ENABLE_ALL:
            for (int i = 0; i < 14; ++i) { axis_state[i] = 2u; }
            fsm = "OD_CONTROL";
            return {true, "ok"};
        case hr::udp::SVC_DISABLE_ALL:
            for (int i = 0; i < 14; ++i) { axis_state[i] = 0u; }
            fsm = "READY";
            return {true, "ok"};
        case hr::udp::SVC_EMERGENCY_STOP:
            fsm = "EMERGENCY_STOP";
            return {true, "ok"};
        case hr::udp::SVC_RESET_FAULT:
            fsm = "READY";
            return {true, "ok"};
        default:
            break;
        }
        int axis = axis_of(motor_id);
        if (motor_id == 0u) { axis = 0; }                 // 0 = all axes (broadcast)
        else if (axis == 0) { return {false, "unknown motor_id " + std::to_string(motor_id)}; }
        if (svc == hr::udp::SVC_SET_RUN_MODE) {
            ++run_mode_sets;
            last_mode_value = static_cast<uint64_t>(value);
            return {true, "ok"};
        }
        if (svc == hr::udp::SVC_ENABLE_AXIS && axis > 0) { axis_state[axis - 1] = 2u; fsm = "OD_CONTROL"; return {true, "ok"}; }
        if (svc == hr::udp::SVC_DISABLE_AXIS && axis > 0) { axis_state[axis - 1] = 0u; return {true, "ok"}; }
        if (svc == hr::udp::SVC_HOME_AXIS || svc == hr::udp::SVC_RESCAN_UID) { return {true, "ok"}; }
        return {false, "unsupported svc " + std::to_string(svc)};
    }

    void tick()
    {
        for (int i = 0; i < 14; ++i) { pos[i] += 0.5f * (target[i] - pos[i]); }
    }

    hr::udp::StateFrame frame(uint64_t ts) const
    {
        hr::udp::StateFrame f{};
        f.timestamp_us = ts;
        f.fsm = fsm_code();
        f.run_mode = 1u;
        f.axes_connected = 14u;
        f.axes_enabled = 0u;
        for (int i = 0; i < 14; ++i) { if (axis_state[i] == 2u) { ++f.axes_enabled; } }
        f.axes_faulted = 0u;
        f.can_ok = true;
        for (int i = 0; i < 15; ++i) {
            auto& a = f.axes[i];
            a.motor_id = MOTORS[i];
            a.connected = i < 14;
            a.axis_state = axis_state[i];
            a.has_fault = false;
            a.channel = i < 7 ? 1u : 2u;
            a.position = pos[i];
            a.velocity = 0.0f;
            a.effort = 0.0f;
        }
        return f;
    }
};

} // namespace

int main(int argc, char** argv)
{
    hr::udp::LinkConfig cfg;
    cfg.bind_addr = "127.0.0.1";
    cfg.listen_port = 17001;
    cfg.state_rate_hz = 50;
    double duration_s = 10.0;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--listen-port")) cfg.listen_port = static_cast<uint16_t>(std::atoi(argv[i + 1]));
        else if (!std::strcmp(argv[i], "--state-rate")) cfg.state_rate_hz = static_cast<uint32_t>(std::atoi(argv[i + 1]));
        else if (!std::strcmp(argv[i], "--duration")) duration_s = std::atof(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--bind")) cfg.bind_addr = argv[i + 1];
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    hr::udp::Link link(cfg);
    if (!link.open()) { return 1; }
    std::printf("[fake] listening %s:%u state %u Hz\n", cfg.bind_addr.c_str(), cfg.listen_port, cfg.state_rate_hz);
    std::fflush(stdout);

    Fake fake;
    const uint64_t t0 = now_us();
    uint64_t next_state = t0, next_tick = t0, next_report = t0 + 2000000u;
    const uint64_t state_period = link.state_period_us();

    while (g_running.load() && now_us() < t0 + static_cast<uint64_t>(duration_s * 1e6)) {
        pollfd pfd{link.fd(), POLLIN, 0};
        (void)::poll(&pfd, 1, 1);
        const uint64_t now = now_us();
        link.drain([&](uint8_t mode, const hr::udp::CommandEntry* e, std::size_t n) { fake.on_cmd(mode, e, n); },
                   [&](uint16_t svc, uint8_t motor_id, float value) { return fake.on_svc(svc, motor_id, value); },
                   now);
        if (now >= next_tick) { fake.tick(); next_tick += 5000u; }
        if (state_period > 0u && now >= next_state) {
            link.send_state(fake.frame(now), now);
            next_state += state_period;
            if (next_state < now) { next_state = now + state_period; }
        }
        if (now >= next_report) {
            const auto s = link.take_stats();
            std::printf("[fake] peer=%s cmd=%llu entries=%llu svc=%llu state=%llu ack=%llu bad=%llu fail=%llu fsm=%s\n",
                        link.peer_text().c_str(), static_cast<unsigned long long>(s.rx_cmd),
                        static_cast<unsigned long long>(s.rx_cmd_entries), static_cast<unsigned long long>(s.rx_svc),
                        static_cast<unsigned long long>(s.tx_state), static_cast<unsigned long long>(s.tx_ack),
                        static_cast<unsigned long long>(s.rx_bad), static_cast<unsigned long long>(s.tx_fail), fake.fsm.c_str());
            std::fflush(stdout);
            next_report += 2000000u;
        }
    }
    {
        const auto s = link.take_stats();
        std::printf("[fake] peer=%s cmd=%llu entries=%llu svc=%llu state=%llu ack=%llu bad=%llu fail=%llu fsm=%s\n",
                    link.peer_text().c_str(), static_cast<unsigned long long>(s.rx_cmd),
                    static_cast<unsigned long long>(s.rx_cmd_entries), static_cast<unsigned long long>(s.rx_svc),
                    static_cast<unsigned long long>(s.tx_state), static_cast<unsigned long long>(s.tx_ack),
                    static_cast<unsigned long long>(s.rx_bad), static_cast<unsigned long long>(s.tx_fail), fake.fsm.c_str());
    }
    std::printf("[fake] exit moves=%llu svc=%llu run_mode_sets=%llu last_mode=%llu\n",
                static_cast<unsigned long long>(fake.moves), static_cast<unsigned long long>(fake.svcs),
                static_cast<unsigned long long>(fake.run_mode_sets), static_cast<unsigned long long>(fake.last_mode_value));
    return 0;
}
