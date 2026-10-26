// Bench demo: connects to every C5-E found on USB, verifies commissioning,
// then spins each motor at the requested speed for a few seconds.
//
// Usage: motor_demo [rpm] [seconds]     (defaults: 100 rpm, 5 s)
//        motor_demo --list              (discover and print only, no motion)

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "motor/nanolib_session.hpp"

namespace {

volatile std::sig_atomic_t keepRunning = 1;

// Shuts every motor down together, however the scope is left.
class MotorGroup {
public:
    ~MotorGroup() { shutdown(); }

    void add(std::unique_ptr<motor::Motor> m) { motors_.push_back(std::move(m)); }
    std::vector<std::unique_ptr<motor::Motor>>& all() { return motors_; }

    void shutdown() noexcept {
        std::vector<motor::Motor*> raw;
        for (auto& m : motors_) raw.push_back(m.get());
        motor::shutdownAll(raw);
    }

private:
    std::vector<std::unique_ptr<motor::Motor>> motors_;
};

void requireRunning(const char* step) {
    if (!keepRunning) throw motor::MotorError(std::string("Interrupted before ") + step + ".");
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, [](int) { keepRunning = 0; });
    std::signal(SIGTERM, [](int) { keepRunning = 0; });

    const bool listOnly = argc > 1 && std::string(argv[1]) == "--list";
    const int rpm = (argc > 1 && !listOnly) ? std::stoi(argv[1]) : 100;
    const int seconds = argc > 2 ? std::stoi(argv[2]) : 5;

    try {
        motor::NanoLibSession session;
        const auto controllers = session.discover();
        if (controllers.empty()) {
            std::cerr << "No controllers found. Check power, USB, and that Plug & Drive Studio is closed.\n";
            return 1;
        }
        for (const auto& c : controllers) {
            std::cout << '[' << c.index << "] " << c.device << "  on  " << c.bus << '\n';
        }

        // Declared after the session so motors are shut down and released first.
        MotorGroup group;
        for (const auto& c : controllers) {
            group.add(session.connectMotor(c.index, "motor" + std::to_string(c.index)));
            auto& m = *group.all().back();
            m.setCancelCheck([] { return !keepRunning; });
            std::cout << m.name() << " firmware " << m.firmwareVersion() << '\n';
            m.verifyCommissioning();
            std::cout << m.name() << " commissioning OK\n";
        }
        if (listOnly) return 0;

        std::cout << "Spin " << group.all().size() << " motor(s) at " << rpm << " rpm for " << seconds
                  << " s. Motors unloaded and secured, hardware stop ready. Type RUN: ";
        std::string confirmation;
        if (!(std::cin >> confirmation) || confirmation != "RUN") {
            std::cout << "Cancelled.\n";
            return 0;
        }

        // Enable all at zero speed first, then command speed only if nothing was interrupted.
        for (auto& m : group.all()) {
            requireRunning("enable");
            m->enable();
        }
        for (auto& m : group.all()) {
            requireRunning("setting the target velocity");
            m->setVelocityRpm(rpm);
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
        while (keepRunning && std::chrono::steady_clock::now() < deadline) {
            for (auto& m : group.all()) {
                m->checkHealthy();
                const auto t = m->telemetry();
                std::cout << m->name() << ": demand=" << t.demandRpm << " actual=" << t.actualRpm
                          << " encoder=" << t.encoderRpm << " hall=" << t.hallRpm << " rpm  pos=" << t.position
                          << "   ";
            }
            std::cout << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        group.shutdown();
        for (auto& m : group.all()) {
            if (m->needsShutdown()) {
                std::cerr << m->name() << " was NOT confirmed disabled. Use the hardware stop.\n";
                return 1;
            }
        }
        std::cout << "Stopped and disabled.\n";
        return 0;
    } catch (const std::exception& error) {
        // MotorGroup's destructor has already zeroed and disabled every motor.
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
