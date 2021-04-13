# Experimental Flight
A flight controller software using the Arduino Framework (Specifically Teensy 4.0) thats designed to be highly modular and for use in highly experimental vehicles and complex control schemes.
## **This project is still under initial development and should not be used even for experimental systems as it will not work and has major bugs.**
## Current project status:
Rough structure is done. Basic modules have been created for testing on actual hardware and implementation for a Starship is underway to develop more modules and test the software. 
## Current goal roadmap:
- [x] Create basic structure and test on hardware.
- [ ] Develope Communication protocols and packet handling for radio control and basestation control.
- [ ] Implement a standard library for Packets with basestation and radio.
- [ ] Improve sensorfusion.
- [ ] Implement Starship vehicle and test.
- [ ] Implement standard modules for common vehicle types.
- [ ] Idk, you tell me.
## TRL: 3
## Installation Commands:
To initialise submodules used: 
```
git submodule update --init --recursive
```
