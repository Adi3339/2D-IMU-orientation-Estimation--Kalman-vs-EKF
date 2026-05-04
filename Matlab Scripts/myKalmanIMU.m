function [x, K, P, x_pred] = myKalmanIMU(acc_x, acc_y, acc_z, gyro_x, gyro_y, time_stamp)

N = length(time_stamp);

dt = zeros(N,1);
for i = 2:N
    dt(i) = time_stamp(i) - time_stamp(i-1);
end

gyro_x = rad2deg(gyro_x);
gyro_y = rad2deg(gyro_y);

roll_acc  = rad2deg(atan2(acc_y, acc_z));
pitch_acc = rad2deg(atan2(-acc_x, sqrt(acc_y.^2 + acc_z.^2)));

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
B(1,1,:) = dt;
B(2,2,:) = dt;
% control matrix,u_k
u_k = zeros(2,N);
u_k(1,:) = (gyro_x).';
u_k(2,:) = (gyro_y).';
% prediction uncertainity matrix
P_pred = zeros(2,2,N);
% P matrix
P = zeros(2,2,N);
% process noise uncertainity(Q)
alpha = 1e6;  % tuning parameter 1e7 is balanced
Q = zeros(2,2,N);
Q(1,1,:) = alpha*(8.9525e-04*dt).^2; % taken after running Gyro_Acc.m
Q(2,2,:) = alpha*(0.0035*dt).^2; % since sensor is same
% mean std-> 0.0022
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
R = [0.4702^2 0;0 0.7534^2];  % std of sensor(accaeleration) vals
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
        P_pred(:,:,k) = P_nod + Q(:,:,k);
    end
    if k > 1
        P_pred(:,:,k) = P(:,:,k-1) + Q(:,:,k-1);
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

end