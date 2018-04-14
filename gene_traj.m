%generate trajectory
clear;clc;

trajfile = 'traj_m';
dt = 0.01;

%--low accurate--%
acc_bias = [5, 5, -4]*0.01 *1; %[mg], m/s^2
gyro_bias = [0.2, 0.15, -0.1]/180*pi *1; %[deg/s], rad/s
acc_noise =  (0.11/sqrt(3600))^2 *1; % [m/s^2/sqrt/sqrt(Hz)], (m/s^2)^2/Hz
gyro_noise = (0.66/sqrt(3600)/180*pi)^2 *1; % [deg/s/sqrt(Hz)], (rad/s)^2/Hz
acc_drift = 0;
gyro_drift = 1;

acc_quantize = 0.01 *1; %m/s^2
gyro_quantize = 0.01 *1; %deg/s
% noise_seeds = [23093 23094 23095 23096 23097 23098];
noise_seeds = randi(1e5,1,6);
%***************************************%

k = 0;
while 1
    t = k*dt/2;
    k = k+1;
    eval(trajfile);
    if k==n
        break
    end
end

cmd.time = (0:dt/2:T)';
cmd.signals.values = [angle,speed];
cmd.signals.dimensions = 6;
sim trajectory

%--drift--%
drift_g = gene_drift(size(imu,1), 3, dt, 40, [10,4], 0.05)*gyro_drift;
imu(:,1:3) = imu(:,1:3)+drift_g;

%--quantize--%
if acc_quantize~=0
    imu(:,4:6) = acc_quantize*round(imu(:,4:6)/acc_quantize);
end
if gyro_quantize~=0
    imu(:,1:3) = gyro_quantize*round(imu(:,1:3)/gyro_quantize);
end