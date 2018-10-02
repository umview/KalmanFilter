%imukf_c.m
clear all;
raw = load('imu.txt');
dt = 0.002;
coe = 14.6884 * dt;
raw = raw';
raw(2,:) = raw(2,:) / coe;
N = length(raw);

Q_angle = 0.4;
Q_gyro = 0.9;
R_angle = 2;
Angle_err = 0;
PCt_0 = 0;
PCt_1 = 0;
E = 0;
K_0 = 0;
K_1 = 0;
t_0 = 0;
t_1 = 0;
Pdot = zeros(1,4);
PP = eye(2);
C_0 = 1.0;

Angle = zeros(1,N);
Gyro = zeros(1,N);
Q_bias = zeros(1,N);

Angle(1) = 0;%raw(1,1);
Gyro(1) = raw(2,1);
for k = 2:N
    Angle(k) = Angle(k-1) + (raw(2,k) - Q_bias(k-1)) * dt;
    Pdot(1) = Q_angle - PP(1,2) - PP(2,1) + PP(2,2) * dt;
    Pdot(2) = -PP(2,2);
    Pdot(3) = -PP(2,2);
    Pdot(4) = Q_gyro;
    PP(1,1) = PP(1,1) + Pdot(1) * dt;
    PP(1,2) = PP(1,2) + Pdot(2) * dt;
    PP(2,1) = PP(2,1) + Pdot(3) * dt;
    PP(2,2) = PP(2,2) + Pdot(4) * dt;
    Angle_err = raw(1,k) - Angle(k);
    PCt_0 = C_0 * PP(1,1);
    PCt_1 = C_0 * PP(2,1);
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PP(1,2);
    PP(1,1) = PP(1,1) - K_0 * t_0;
    PP(1,2) = PP(1,2) - K_0 * t_1;
    PP(2,1) = PP(2,1) - K_1 * t_0;
    PP(2,2) = PP(2,2) - K_1 * t_1;
    Angle(k) = Angle(k) + K_0 * Angle_err;
    Q_bias(k) = Q_bias(k) + K_1 * Angle_err;
    Gyro(k) = raw(2,k) - Q_bias(k);
end
%plot(raw(1,:));
hold on;
plot(Angle);
%plot(Q_bias);
%plot(raw(2,:)-Q_bias);

