#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <numbers>
#include <string_view>

struct MotorSpec {
    std::string_view name;
    double R_ohm;       // Phase-to-phase / effective winding resistance (Ohm)
    double L_henry;     // Winding inductance (H)
    double kv_rpm_per_v;
    double voltage_limit;

    [[nodiscard]] constexpr double ke() const noexcept {
        return 60.0 / (2.0 * std::numbers::pi * kv_rpm_per_v);
    }
    [[nodiscard]] constexpr double kt() const noexcept { return ke(); }
};

struct SimResult {
    double final_dist_m{};
    double max_speed_mps{};
    double peak_current_a{};
    double min_vbus_v{};
    bool drv8313_overcurrent{};
};

SimResult run_coupled_sim(const MotorSpec& motor, double duration_s = 2.5) {
    constexpr double dt = 1e-5; // 10 us timestep (100 kHz)
    constexpr double v_bat_oc = 11.1;
    constexpr double r_bat = 0.06;
    constexpr double c_bulk = 1000e-6;
    constexpr double mass_kg = 0.92;
    constexpr double wheel_r = 0.048;
    constexpr double g = 9.81;

    double v_bus = v_bat_oc;
    double i_motor = 0.0;
    double x = 0.0;
    double v = 0.0;
    double wheel_angle = 0.0;

    SimResult res{.min_vbus_v = v_bat_oc};

    const int steps = static_cast<int>(duration_s / dt);
    for (int s = 0; s < steps; ++s) {
        const double t = s * dt;

        // 1. AS5600 12-bit quantized angle (4096 ticks/rev)
        constexpr double two_pi = 2.0 * std::numbers::pi;
        const int ticks = static_cast<int>(std::fmod(wheel_angle, two_pi) / two_pi * 4096.0);
        [[maybe_unused]] const double sensed_angle = (ticks / 4096.0) * two_pi;

        // 2. SimpleFOC voltage_limit command clamped to available bus voltage
        const double v_cmd = std::min(motor.voltage_limit, v_bus);

        // 3. Electrical RL + Back-EMF ODE: L * di/dt = V_cmd - I*R - Ke*omega
        const double omega = v / wheel_r;
        const double back_emf = motor.ke() * omega;
        const double di_dt = (v_cmd - i_motor * motor.R_ohm - back_emf) / motor.L_henry;
        i_motor += di_dt * dt;

        if (std::abs(i_motor) > 2.5) res.drv8313_overcurrent = true;
        res.peak_current_a = std::max(res.peak_current_a, std::abs(i_motor));

        // 4. 3S 18650 + 1000uF Bulk Cap Bus Voltage Dynamics (4 motors total)
        const double i_total = 4.0 * std::max(0.0, i_motor) * (v_cmd / std::max(0.1, v_bus));
        const double i_bat = (v_bat_oc - v_bus) / r_bat;
        v_bus += ((i_bat - i_total) / c_bulk) * dt;
        res.min_vbus_v = std::min(res.min_vbus_v, v_bus);

        // 5. Mechanical: 4x Wheel Torque -> Buggy Motion (with 20mm curb at 0.6m & 15 deg ramp at 1.5m)
        const double torque_total = 4.0 * motor.kt() * i_motor;
        const double f_drive = torque_total / wheel_r;

        double slope_rad = 0.0;
        if (x >= 0.58 && x <= 0.62) slope_rad = 0.35;      // 20 mm curb step resistance
        else if (x >= 1.50 && x <= 2.10) slope_rad = 0.2618; // 15 deg ramp

        const double f_drag = 0.15 * v + mass_kg * g * std::sin(slope_rad) + 0.04 * mass_kg * g;
        const double accel = (f_drive - f_drag) / mass_kg;

        v = std::max(0.0, v + accel * dt);
        x += v * dt;
        wheel_angle += (v / wheel_r) * dt;
        res.max_speed_mps = std::max(res.max_speed_mps, v);
        (void)t;
    }

    res.final_dist_m = x;
    return res;
}

int main() {
    constexpr MotorSpec m2804{"2804 100KV (6.5 Ohm, V_lim=11.1V)", 6.5, 2.5e-3, 100.0, 11.1};
    constexpr MotorSpec m5010_high{"5010 360KV (0.2 Ohm, V_lim=1.2V)", 0.20, 35e-6, 360.0, 1.2};
    constexpr MotorSpec m5010_safe{"5010 360KV (0.2 Ohm, V_lim=0.45V)", 0.20, 35e-6, 360.0, 0.45};

    std::cout << "=== C++23 Coupled Electro-Mechanical Buggy Simulator (2.5s Run) ===\n\n";
    for (const auto& m : {m2804, m5010_high, m5010_safe}) {
        const auto r = run_coupled_sim(m);
        std::cout << std::format(
            "{}\n"
            "  Distance: {:.2f} m | Max Speed: {:.2f} m/s | Peak Current: {:.2f} A | Min V_bus: {:.2f} V | DRV8313 Overcurrent (>2.5A): {}\n\n",
            m.name, r.final_dist_m, r.max_speed_mps, r.peak_current_a, r.min_vbus_v,
            r.drv8313_overcurrent ? "YES (TRIP!)" : "NO (SAFE)");
    }
    return 0;
}
