# Experimental Flight
A flight controller software using the Arduino Framework (Specifically Teensy 4.0) thats designed to be highly modular and for use in highly experimental vehicles and complex control schemes.
## **This project is still under initial development and should not be used even for experimental systems as it will not work and has major bugs.**
## Current goal roadmap:
- [x] Create basic structure and test on hardware.
- [ ] Develop Communication protocols and packet handling for radio control and basestation control.
- [ ] Implement a standard library for Packets with basestation and radio.
- [ ] Add position/velocity and acceleration to Sensorfusion using accelerometer and GPS 
- [ ] Release first alpha version. Yay!
- [ ] Improve sensorfusion algorithm and auto-adjustment algorithm for filter factors.
- [ ] Implement Starship vehicle and test.
- [ ] Implement standard modules for common vehicle types.
- [ ] Idk, you tell me...
## TRL: 3
## Installation Commands:
To initialise submodules used: 
```
git submodule update --init --recursive
```
