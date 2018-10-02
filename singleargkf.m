clear;
raw = load('imu.txt');
dt = 0.002;
coe = 14.6884 * dt;
raw = raw';
raw(2,:) = raw(2,:) / coe;
N = length(raw);
X = zeros(1,N);
K = zeros(1,N);
%X(1,1) = 0;%raw(1,1);
P = eye(1);
I = eye(1);
Q = 1;
R = 500;
for k = 2:N
   X(k) = X(k-1) + raw(2,k) * dt;
   P = P + Q;
   K(k) = P / (P + R);
   X(k) = X(k) + K(k) * (raw(1,k) - X(k));
   P = (I - K(k)) * P;
end

plot(X);