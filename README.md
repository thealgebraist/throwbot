# Throwbot — 20 cm Gearless Electromagnetic Outrunner Buggy

Self-contained engineering dossier, blueprints, SPICE circuits, C++23 electro-mechanical simulators, Wokwi ESP32-S3 wiring project, and Gazebo Sim (`SDF 1.8`) world for a `200 × 160 × 96 mm` 4-wheel invertible gearless outrunner buggy.

## Contents

* **`outrunner_buggy_dossier.pdf` / `outrunner_buggy_dossier.tex`** — Complete engineering dossier (3 BOM tiers, spectral/matrix derivations, 5-level backup architecture, industrial hardening, and embedded blueprints/renders).
* **`figures/`** — High-resolution renders and labeled blueprints (`fig_optionB_blueprint.png`, `fig_budget_blueprint.png`, `fig_extreme_blueprint.png`, `fig_seethrough_blueprint.jpg`, etc.).
* **`buggy_sim.cpp` & `extreme_buggy_sim.cpp`** — C++23 coupled electro-mechanical simulators (`2804 100KV` vs `5010 360KV`, `-30°C` cold stall, `4 m` drop, `100 kg` crush, 1-wheel-dead limp mode).
* **`buggy_power_and_motor.cir` & `extreme_rugged_circuit.cir`** — `ngspice` circuit simulations.
* **`model.sdf`, `buggy_test_world.sdf`, `run_gazebo.sh`** — Gazebo Sim (`SDF 1.8`) 4WD skid-steer model & test world.
* **`wokwi/`** — `ESP32-S3` + `MPU6050` + PWM test wiring (`diagram.json`, `sketch.ino`).

## Quick Run

```zsh
# Compile & run C++23 simulators
clang++ -std=c++23 -O2 buggy_sim.cpp -o buggy_sim && ./buggy_sim
clang++ -std=c++23 -O2 extreme_buggy_sim.cpp -o extreme_buggy_sim && ./extreme_buggy_sim

# Run ngspice circuit simulations
ngspice -b buggy_power_and_motor.cir
ngspice -b extreme_rugged_circuit.cir

# Launch Gazebo Sim (macOS -s / -g split)
./run_gazebo.sh
```
