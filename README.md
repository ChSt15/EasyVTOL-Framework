# KraftFramework
A flight controller framework using the Arduino Framework (Current development is only on the Teensy 4.0) thats designed to be highly modular and for use in highly experimental vehicles and complex systems, without many compromises to performance.
The philosophy behind the architecture is fault handling at runtime and compilation time through automation of base classes, while still keeping direct communication between modules.
This is of great importance as testing of modules and switching between them during flight is possible with this modular design and greatly simplifies programming, and a failure during switching could easily cause a crash. 
## **This project is still under initial development. Huge structure and architecture changes are very likely to happen resulting in many broken things.**
## Current goal roadmap:
- [x] Create basic structure and test on hardware.
- [x] Develop Communication protocols and packet handling for radio control and basestation control.
- [x] Implement a standard library for Packets with basestation and radio.
- [x] Add position/velocity and acceleration to Sensorfusion using accelerometer and GPS.
- [ ] Improve complementary sensor fusion algorithm.
- [x] Add topic system to all modules.
- [x] Add new message type system to message structure.
- [ ] Add data message connection system to KraftKommunication. 
- [ ] Create topic router.
- [ ] Prepare for first alpha version to be released.
- [ ] Released first alpha version. Yay!
- [ ] Improve sensorfusion algorithm and auto-adjustment algorithm for filter factors.
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

