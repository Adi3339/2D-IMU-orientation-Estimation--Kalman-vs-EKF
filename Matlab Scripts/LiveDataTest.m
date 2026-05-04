% Live sensor values from Matlab mobile
% mobiledevlist
m = mobiledev();
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
m.SampleRate = 100;

while true
    
    % Read latest sensor values
    acc = m.Acceleration;
    gyro = m.AngularVelocity;
    
    % Skip if no new data
    if isempty(acc) || isempty(gyro)
        continue;
    end
    
    % Take latest sample
    acc_k = acc(end, :);     % [ax; ay; az]
    gyro_k = gyro(end, :);   % [gx; gy; gz]

    fuse = imufilter('SampleRate', 100, 'AccelerometerNoise',0.0826, 'GyroscopeNoise',0.1188); 
    % std in accel(x,y,z) and gyro (x,y,z) sensor values
    q = fuse(acc_k, gyro_k);
    eul = eulerd(q,'ZYX','frame');
    imuf_roll = eul(:,3);
    imuf_pitch = eul(:,2);
    
    % Time step (assume fixed for now)
    dt = 1 / m.SampleRate;
    
    % ---- CALL YOUR EKF STEP FUNCTION ----
    [x,K, P,x_pred] = myKalmanIMU(acc_k(:,1), acc_k(:,2), acc_k(:,3), ...
        gyro_k(:,1), gyro_k(:,2) ,dt);
    
    % Display / plot
    roll  = x(1);
    pitch = x(2);
    
    fprintf("myRoll: %.2f | myPitch: %.2f      |" + ...
        "Matlab_roll: %.2f | Matlab_pitch: %.2f\n", ...
        roll, pitch, imuf_roll, imuf_pitch);
    % using this form to match ZYX convention of euler
    
    pause(0.01);  % prevent CPU overload
end
% m.Logging = 1;
% pause(80);
% m.Logging = 0;


% code part if data is logged instead of live stream
[acc, time_stamp_2] = accellog(m);
[gyro,time_stamp_1] = angvellog(m);
if size(time_stamp_2,1) > size(time_stamp_1,1)
    a = size(time_stamp_2,1) - size(time_stamp_1,1);
    time_stamp_2 = time_stamp_2(1:end-a,:);
    acc = acc(1:end-a,:);
   
end





%% All three filter compared visually
time_stamp = Acceleration.Timestamp;
gyro_x = AngularVelocity.X;
gyro_y = AngularVelocity.Y;
gyro_z = AngularVelocity.Z;

[x_k,K_k,P_k,x_pred_k] = myKalmanIMU(acc(:,1),acc(:,2),acc(:,3),gyro(:,1),gyro(:,2),time_stamp_1);

[x,K,P,x_pred] = myEKF(acc(:,1),acc(:,2),acc(:,3),gyro(:,1),gyro(:,2),time_stamp_1);

plot(time_stamp_1,x_k(1,:) ,'-', ...
    'Color', [0 0.447 0.741], 'LineWidth', 2); hold on;
plot(time_stamp_1, rad2deg(x(1,:)), '-', ...
    'Color', [0.85 0.325 0.098], 'LineWidth', 1.8); hold on;
 

legend('Roll- Matlab filter', 'Roll- Kalman filter');
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Linear acceleration and free fall test');
grid on;

