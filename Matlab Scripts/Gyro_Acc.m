load("Gyro_acc_values");
acc_x = Acceleration.X;
acc_y = Acceleration.Y;
acc_z = Acceleration.Z;

time_stamp = Acceleration.Timestamp;
gyro_x = AngularVelocity.X;
gyro_x = rad2deg(gyro_x);
gyro_y = AngularVelocity.Y;
gyro_y = rad2deg(gyro_y);
gyro_z = AngularVelocity.Z;
gyro_z = rad2deg(gyro_z);

freq = 100; % sampling frequency, Hz

%% Seperate out first segment of the data to get Q and R mat
% Process to get Q matrix: represents uncertainty in prediction
% (in this case- gyro integraton values). std_angle = std_gyro*dt
std_acc_x = std(acc_x(1:3000));
std_acc_y = std(acc_y(1:3000));
std_acc_z = std(acc_z(1:3000));

std_gyro_x = std(gyro_x(1:3000));  % flat for 3s(3k samples)
std_gyro_y = std(gyro_y(1:3000));
std_gyro_z = std(gyro_z(1:3000));
% define roll and pitch (wrt acc values)
roll_acc = rad2deg(atan2(acc_y, acc_z));
pitch_acc = rad2deg(atan2(-acc_x, sqrt(acc_y.*acc_y + acc_z.*acc_z)));
std_roll = std(roll_acc(1:3000));
std_pitch = std(pitch_acc(1:3000));
% plot(Acceleration.Timestamp, pitch)
%% Kalman filter implemantation- Prediction Matrices
% calculating del_t:
N = length(time_stamp); 
del_t = zeros(N,1);
for i=2:N
    del_t(1) = 0; 
    del_t(i) = time_stamp.Second(i) - time_stamp.Second(i-1);
end
% initial estimates x_nod, P_nod
x_nod = [roll_acc(1);pitch_acc(1)];
P_nod = [10 0;0 10];   % subject to change/optimize->[10 0;0 10]
% state matrix
x = zeros(2,N);
% prediction matrix x_pred
x_pred = zeros(2,N);
% F- matrix
F = [1 0; 0 1];
% B matrix- with carying del_t
B = zeros(2,2,N);
B(1,1,:) = del_t;
B(2,2,:) = del_t;
% control matrix,u_k
u_k = zeros(2,N);
u_k(1,:) = gyro_x;
u_k(2,:) = gyro_y;
% prediction uncertainity matrix
P_pred = zeros(2,2,N);
% P matrix
P = zeros(2,2,N);
% process noise uncertainity(Q)
alpha = 100;  % tuning parameter 1e7 is balanced
Q = zeros(2,2,N);
Q(1,1,:) = alpha*(std_gyro_x*del_t).^2;
Q(2,2,:) = alpha*(std_gyro_y*del_t).^2;
% measurement matrices
% innovation matrix y
y = zeros(2,N);
% measuremnet vector z
z = zeros(2,N);
z(1,:) = roll_acc;
z(2,:) = pitch_acc;
% observation matrix H
H = [1 0;0 1];
% noice uncertainity R
R = [std_roll^2 0;0 std_pitch^2];
% innovation covariance S
S = zeros(2,2,N);
% kalman gain K
K = zeros(2,2,N);
for k=1:N
    % prediction eqution
    if k == 1
        x_pred(:,k) = F*x_nod + B(:,:,k)*u_k(:,k);
    end
    if k > 1
        x_pred(:,k) = F*x(:,k-1) + B(:,:,k-1)*u_k(:,k-1);
    end
       
    % covariance equation
    if k==1
        P_pred(:,:,k) = P_nod + q(:,:,k);
    end
    if k > 1
        P_pred(:,:,k) = P(:,:,k-1) + q(:,:,k-1);
    end
    
    % innovation eqn
    y(:,k) = z(:,k) - H*x_pred(:,k);
    
    % innovation covariance eqn
    S(:,:,k) = H*P_pred(:,:,k)*H + R;

    % kalman gain eqn
    K(:,:,k) = P_pred(:,:,k)*H / S(:,:,k);

    % state update
    x(:,k) = x_pred(:,k) + K(:,:,k)*y(:,k);

    % covariance update
    P(:,:,k) = (eye(2)-(K(:,:,k)*H))*P_pred(:,:,k);
end
    figure;
plot(time_stamp, pitch_acc, 'r', 'LineWidth', 1.5); hold on;
plot(time_stamp, x(2,:), 'b', 'LineWidth', 1);

legend('Roll (Accelerometer)', 'Roll (Kalman Filter)');
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Roll: Accelerometer vs Kalman Filter');
grid on;
% degubbing functions
figure(1);
    plot(time_stamp, x(1,:))
    figure(3);
    plot(time_stamp, roll_acc)
    figure(2);
    plot(time_stamp, x(2,:))
  
    plot(time_stamp,squeeze(K(1,1,:)))
    plot(time_stamp,squeeze(y(1,:)))
mean(K(1,1,:))
m = mobiledev; mobiledevlist
r = rad2deg(atan2(5.5, 8))
p = rad2deg(atan2(0.1, sqrt(5.5*5.5 + 8*8)))
