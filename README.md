# Unscented-Kalman-Flter
Implementation of UKF on a CTRV (Constant Turn Rate and Velocity) process model for object tracking.
The UKF is a powerful technique for performing recursive nonlinear estimations. Compared to Extended
Kalman filter, UKF uses a derivative-free approach. UKF is also more accurate than the Extended Kalman Filter 
and has an equivalent computational complexity.

[More on UKF](https://www.pdx.edu/biomedical-signal-processing-lab/sites/www.pdx.edu.biomedical-signal-processing-lab/files/ukf.wan_.chapt7_.pdf)

[EKF implimentation of a similar project](https://github.com/askmuhsin/extended-kalman-flter)

---

# Build Instruction
1. Clone this repo and cd into it.
2. `mkdir build && cd build`
3. `cmake ..` 
4. `make`
5. Run : `./UnscentedKF`

Note: This project requires the Udacity open source simulator : [Udacity term 2 sim](https://github.com/udacity/self-driving-car-sim/releases)

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
  ---
  
  ## Result
  
  ## TODO
  - [x] finish control flow
  - [x] achieve acceptable rmse
