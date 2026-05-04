## 2D-IMU-orientation-Estimation--Kalman-vs-EKF
This project showcases two custom filters — a 2D Kalman filter and a 2D Extended Kalman Filter (EKF) — against MATLAB’s built‑in imufilter for estimating roll and pitch from accelerometer + gyroscope data.
The goal is to understand:
how well simple vs nonlinear filters perform for 2D tilt estimation,
how they behave under extreme motion,
and what the fundamental limitations of acc+gyro fusion are.
*All experiments use real data collected from MATLAB Mobile on a smartphone.

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

## Experiments
1. Long Flat Test (8 minutes)
- Purpose: check drift, bias and static stability
# Results:
- Kalman: stable, no drift
- EKF: stable, slightly better than Kalman(0.4° closer to Matlab's imufilter)
 Conclusion: Both filters behave correctly in the easiest experiment.

2. Extreme Orientations
- right/left tilt, upward/downward tilt, reverse flat, mixed roll/pitch combinations
# Results:
- Kalman matches imufilter extremely well
- EKF also matches imufilter when tuned properly
- Both filters correctly identify reverse-flat(roll ~180°)
- At pitch ~90°, both filters show roll drift/instability

Why roll becomes unstable at pitch = ±90°
This is a mathematical singularity of Euler angles:
At pitch = ±90°, the rotation sequence loses one degree of freedom
Roll and yaw become coupled
Small noise in yaw → large apparent changes in roll
This is not a filter bug — it’s Euler angle ambiguity
Even MATLAB’s imufilter shows this behavior when converted to Euler angles.

Conclusion: Euler roll is not physically meaningful at ±90° pitch. All filters struggle here because the representation itself breaks.

3. Linear Acceleration & Free Fall(Adversarial Test)
- Purpose: show the limitations of using acc + gyro for tilt
- Scenario: 30 seconds of linear acceleration while keeping device flat
  30 seconds of random fast motion
  10 seconds flat
  Free fall with spin (multiple rotations)
  
# Experiments & Results: 
1. Static Test (8 min)
Goal: Evaluate drift and stability
Kalman: stable, no drift
EKF: stable, slightly closer to MATLAB imufilter

Conclusion: Both filters perform well under static conditions.

2. Extreme Orientations
Goal: Test behavior under large roll/pitch angles
Both filters closely match imufilter
Correctly detect reverse-flat (~180° roll)
Instability observed at pitch ≈ ±90°

Note:
At ±90° pitch, Euler angles become singular (gimbal lock). Roll and yaw are no longer independent, so instability is expected across all methods.

3. Dynamic Motion & Free Fall
Goal: Evaluate robustness under acceleration and unobservable conditions
Kalman: tracks motion well, behaves like gyro integration when accel is unreliable
EKF: matches imufilter, but sensitive to Q tuning (noise vs. bias tradeoff)
imufilter: conservative, rejects accel when invalid

Observation:
During strong acceleration or free fall, gravity cannot be isolated from accelerometer data:
a_measured = g + a_linear
- Tilt becomes unobservable
- All filters degrade

## Conclusions
# For 2D tilt only, the Kalman filter is the best practical choice
- Much simpler
- Robust
- Covers ground truth extremely well
- Less sensitive to tuning than EKF

# The EKF is mathematically correct but not worth the complexity for 2D
- More sensitive and noisy
- Underestimates angles unless tuned aggressively

# Euler angles break at pitch = ±90°
- Roll becomes undefined
- This is a representation problem, not a filter problem. Quaternion solve this problem

# Linear acceleration & free fall showcase the limits of acc + gyro
- Gravity direction becomes unobservable
- Filters diverge
- imufilter is conservative by design

## Tools used-
1. Matlab
2. Matlab Mobile App
