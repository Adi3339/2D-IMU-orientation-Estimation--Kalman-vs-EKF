function [x, K, P, x_pred] = myEKF(acc_x, acc_y, acc_z, gyro_x, gyro_y, time_stamp)

N = length(time_stamp);

dt = zeros(N,1);
for i = 2:N
    dt(1) = 0;
    dt(i) = time_stamp(i) - time_stamp(i-1);
end



g = 9.81;

x_nod = [0;0];
P_nod = [0.0012 0;0 0.0012];   % subject to change/optimize->[10 0;0 10]
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
u_k(1,:) = (gyro_x).';  % transpose of matrix to match the input data size
u_k(2,:) = (gyro_y).';
% prediction uncertainity matrix
P_pred = zeros(2,2,N);
% P matrix
P = zeros(2,2,N);
% process noise uncertainity(Q)
alpha = 1e3;  
Q = zeros(2,2,N);
Q(1,1,:) = alpha*(8.0147e-07*dt); % taken after running Gyro_Acc.m
Q(2,2,:) = alpha*(0.0035*dt).^2; % since sensor is same
% mean std-> 0.0022
% measurement matrices
% innovation matrix y
y = zeros(3,N);
% measuremnet vector z
z = zeros(3,N);
z(1,:) = (acc_x).';
z(2,:) = (acc_y).';
z(3,:) = (acc_z).';
% observation matrix h
h = zeros(3,N);

% noice uncertainity R
R = diag([0.0172 0.0066 0.0012]); % calculated from flat data sample
% variance of all three acc values

% innovation covariance S
S = zeros(3,3,N);
% kalman gain K
K = zeros(2,3,N);
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
    h(1,k) = -g*sin(x_pred(2,k));
    h(2,k) = g*cos(x_pred(2,k))*sin(x_pred(1,k));
    h(3,k) = g*cos(x_pred(1,k))*cos(x_pred(2,k));
    y(:,k) = z(:,k) - h(:,k);

    % innovation covariance eqn
    % Jacobian of h(x_pred)
    H = zeros(3,2,N);
    H(1,1,k) = 0;
    H(1,2,k) = -g*cos(x_pred(2,k));
    H(2,1,k) = g*cos(x_pred(1,k))*cos(x_pred(2,k));
    H(2,2,k) = -g*sin(x_pred(2,k))*sin(x_pred(1,k));
    H(3,1,k) = -g*cos(x_pred(2,k))*sin(x_pred(1,k));
    H(3,2,k) = -g*cos(x_pred(1,k))*sin(x_pred(2,k));
    
 
    S(:,:,k) = H(:,:,k) *P_pred(:,:,k) * (H(:,:,k).') + R;

    % kalman gain eqn
    K(:,:,k) = P_pred(:,:,k) *(H(:,:,k).') / S(:,:,k);

    % state update
    x(:,k) = x_pred(:,k) + K(:,:,k)*y(:,k);

    % covariance update
    P(:,:,k) = (eye(2)-(K(:,:,k)*H(:,:,k)))*P_pred(:,:,k);

    % wrap 
    %x(1,k) = wrapToPi(x(1,k));
    %x(2,k) = wrapToPi(x(2,k));
    %x_pred(1,k) = wrapToPi(x_pred(1,k));
    %x_pred(2,k) = wrapToPi(x_pred(2,k));
end

end

