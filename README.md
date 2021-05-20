# KraftFramework
A flight controller framework using the Arduino Framework (Current development is only on the Teensy 4.0) thats designed to be highly modular and for use in highly experimental vehicles and complex system, without many compromises to efficiency or performance.
## **This project is still under initial development and should not be used even for experimental systems as it will not work and has major bugs.**
## Current goal roadmap:
- [x] Create basic structure and test on hardware.
- [x] Develop Communication protocols and packet handling for radio control and basestation control.
- [x] Implement a standard library for Packets with basestation and radio.
- [ ] Add position/velocity and acceleration to Sensorfusion using accelerometer and GPS.
- [ ] Test and tune GNSS position correction for complementary sensorfustion
- [ ] Prepare for first alpha version to be released.
- [ ] Release first alpha version. Yay!
- [ ] Improve sensorfusion algorithm and auto-adjustment algorithm for filter factors.
- [ ] Implement Starship vehicle and test.
- [ ] Implement standard modules for common vehicle types. (E.g. Quadcopter, Plane)
- [ ] Implement Kalman filter for navigation.
- [ ] Optimise everything.
- [ ] Clean code.
- [ ] Improve comments in code.
- [ ] Create documentation.
- [ ] Idk, you tell me...
## Installation Commands:
To initialise submodules used: 
```
git submodule update --init --recursive
```
## ToDo:
- [ ] Add new data containers.
- [ ] Migrate to single universal time system. E.g. NOW() and returns runtime int64_t in nanoseconds. Should also solve problem with overflow. This can later be used to simulate modules.
- [ ] Add buffer with queue, stack, sorting, median, average, deviation calculations.
- [ ] Add error calculation system for measurements and sensor fusion. This should make error calculation automatic.

