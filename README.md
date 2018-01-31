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
  The make file runs without any errors.
  The RMSE for dataset 1 is as follows :
  
  Input |   MSE  
  ----- | -------
   px   | 0.0701
   py   | 0.0839 
   vx   | 0.3446
   vy   | 0.2293
     
  The RMSE for same dataset running [EKF](https://github.com/askmuhsin/extended-kalman-flter) :
  
  Input |   MSE  
  ----- | -------
   px   | 0.0974 
   py   | 0.0855 
   vx   | 0.4517 
   vy   | 0.4404 
   
   NIS (normalized innovation squared) was used for optimizing the noise parameters. 
   NIS of liadar and radar measurements visualized:
   ![LIDAR-NIS](https://github.com/askmuhsin/unscented-kalman-filter/blob/master/NIS_visualize/lidar_1.png)
   <br></br>
   ![RADAR_NIS](https://github.com/askmuhsin/unscented-kalman-filter/blob/master/NIS_visualize/radar_1.png)
   <br></br>
   ![ScreenCapture](https://github.com/askmuhsin/unscented-kalman-filter/blob/master/screenshot.png)
   
  
  ## TODO
  - [x] finish control flow
  - [x] achieve acceptable rmse
