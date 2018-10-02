clear;
% Native Matrix Kalman Filter
raw = load('imu.txt');
dt = 0.002;
coe = 14.6884 * dt;
raw = raw';
raw(2,:) = raw(2,:) / coe;
N = length(raw);
X = zeros(2,N);
K = zeros(2,N);
X(1,1) = 0;%raw(1,1);


%Q_angle = 0.45;
Q_gyro = 0.0001 * dt;
R_angle = 1.7 / dt;

Q_angle = 0.7;
%Q_gyro = 0.0001 * dt;
%R_angle = 1.7 / dt;


R = ones(1);
R(1,1) = R_angle;
Q = [Q_angle,0;0,Q_gyro];
A = [1,-dt;0,1];
B = [dt;0];
P = eye(2);
H = [1,0];
I = eye(2);
Z = raw;
for k=2:N
    X(:,k) = A * X(:,k-1) + B * Z(2,k);
    P = A * P * A' + Q;
    K(:,k) = P * H' * (H * P * H' + R)^-1;
    X(:,k) = X(:,k) + K(:,k) * (Z(1,k) - H * X(:,k));
    P = (I - K(:,k) * H) * P;
end

%%%%%%%%%%%%    C-form KalmanFilter %%%%%%%%%%%%%%%%%%%%
raw = load('imu.txt');
dt = 0.002;
coe = 14.6884 * dt;
raw = raw';
raw(2,:) = raw(2,:) / coe;
N = length(raw);
%Q_angle = 0.4;
Q_gyro = Q_gyro / dt;
R_angle2 = R_angle * dt;
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
    E = R_angle2 + C_0 * PCt_0;
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
%%%%%%%%%%%% SingleArgKF %%%%%%%%%%%
raw = load('imu.txt');
dt = 0.002;
coe = 14.6884 * dt;
raw = raw';
raw(2,:) = raw(2,:) / coe;
N = length(raw);
X2 = zeros(1,N);
K2 = zeros(1,N);
%X(1,1) = 0;%raw(1,1);
P = eye(1);
I = eye(1);
Q = Q_angle;
R = R_angle;
for k = 2:N
   X2(k) = X2(k-1) + raw(2,k) * dt;
   P = P + Q;
   K2(k) = P / (P + R);
   X2(k) = X2(k) + K2(k) * (raw(1,k) - X2(k));
   P = (I - K2(k)) * P;
end

%%%%%%%%%%%% PI Filter %%%%%%%%%%%%%%%
dt = 0.002;
%coe = 14.6884 *dt;
Kp = 20;
Ki = 0.003;
raw = load('imu.txt');
raw = raw';
N = length(raw);
Y = zeros(2,N);
for k = 2:N
    Y(2,k) = Y(2,k-1) + (raw(1,k) - Y(1,k-1));
    Y(1,k) = Y(1,k-1) + (raw(2,k) + Kp * (raw(1,k) - Y(1,k-1) - raw(2,k) * dt) + Ki * Y(2,k)) * dt;
    %Y(1,k) = Y(1,k-1) + (raw(2,k) + Kp * (raw(1,k) - Y(1,k-1)) + Ki * Y(2,k)) * dt;

end

%%%%%%%%%%%%    Balance Filter   %%%%%%%%%%%%%%

dt=0.002;
coe=14.6884 *dt;
K2=0.008;
line=load('imu.txt');
line(:,2)=line(:,2)/coe;
line(1,3)=line(1,1);
line(1,4)=0;%line(1,1);
for i=2:length(line)
    line(i,3)=line(i-1,3)-dt*line(i,2);
    line(i,4)=(line(i-1,4)-dt*line(i,2))*(1-K2)+line(i,1)*K2;
end

%%%%%%%%%%%%%%%%%%%%%%% Plot %%%%%%%%%%%%%%
%plot(raw(1,:));
hold on;
plot(raw(1,:));
%plot(raw(2,:));
plot(X(1,:));
%plot(X(2,:));
%plot(raw(2,:)-X(2,:));
%plot(Angle);
%plot(Q_bias);
%plot(raw(2,:)-Q_bias);
%plot(line(:,4));
%plot(X(1,:)-Angle);
%plot(X(1,:)-line(:,4)');
%plot(K(1,:));
%plot(K(2,:));
plot(X2);
%plot(Y(1,:));
%plot(Y(2,:));