## 2D-IMU-orientation-Estimation--Kalman-vs-EKF
This project showcases two custom filters — a 2D Kalman filter and a 2D Extended Kalman Filter (EKF) — against MATLAB’s built‑in imufilter for estimating roll and pitch from accelerometer + gyroscope data.
The goal is to understand:
how well simple vs nonlinear filters perform for 2D tilt estimation,
how they behave under extreme motion,
and what the fundamental limitations of acc+gyro fusion are.
*All experiments use real data collected from MATLAB Mobile on a smartphone.

A live demo video shows how the filter estimates vs Matlab's filter:
https://github.com/user-attachments/assets/aaf969d5-d8b9-4232-bf61-81c20be9c1a9

## Implimented Filters
1. 2D Kalman Filter(KF)
- State: roll, pitch
- Measurement: atan2(acc)
- Linear Model
- Very Stable, robust

2. 2D Extended Kalman Filter(EKF)
- State: roll, pitch
- Measurement: full nonlinear gravity model, uses Jacobian
- More mathematically correct, but more sensitive

3. MATLAB's Filter(reference)
- Full 3D quaternion filter
- Used as 'ground truth' reference for comparison
  
## Experiments & Results: 
1. Static Test (8 min)
Goal: Evaluate drift and stability.
Kalman: stable, no drift.
EKF: stable, slightly closer to MATLAB imufilter.

   Conclusion: Both filters perform well under static conditions.

2. Extreme Orientations
Goal: Test behavior under large roll/pitch angles.
Both filters closely match imufilter.
Correctly detect reverse-flat (~180° roll).
Instability observed at pitch ≈ ±90°.
Note:
At ±90° pitch, Euler angles become singular (gimbal lock). Roll and yaw are no longer independent, so instability is expected across all methods.

3. Dynamic Motion & Free Fall
Goal: Evaluate robustness under acceleration and unobservable conditions.
Kalman: tracks motion well, behaves like gyro integration when accel is unreliable.
EKF: matches imufilter, but sensitive to Q tuning (noise vs. bias tradeoff).
imufilter: conservative, rejects accel when invalid.
*Note- for more detailed explainations and plots refer to results/

   Observation:
 During strong acceleration or free fall, gravity cannot be isolated from accelerometer data:
 a_measured = g + a_linear
 - Tilt becomes unobservable
 - All filters degrade

## Conclusions
 For 2D tilt only, the Kalman filter is the best practical choice
- Much simpler
- Robust
- Covers ground truth extremely well
- Less sensitive to tuning than EKF

 The EKF is mathematically correct but not worth the complexity for 2D
- More sensitive and noisy
- Underestimates angles unless tuned aggressively

 Euler angles break at pitch = ±90°
- Roll becomes undefined
- This is a representation problem, not a filter problem. Quaternion solve this problem

 Linear acceleration & free fall showcase the limits of acc + gyro
- Gravity direction becomes unobservable
- Filters diverge
- imufilter is conservative by design

## Tools used
1. Matlab
2. Matlab Mobile App
