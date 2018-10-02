%Kalman filter for tempture
clear all;
N = 300;
Q=4e-4;
R=0.5*0.5;
x=zeros(1,N);
out=zeros(1,N);
tempture = 27 * ones(1,N);%
tempture_noise = sqrt(Q) * randn(1,N);
real_tempture = tempture + tempture_noise;
sample_tempture = real_tempture + sqrt(R) * randn(1,N);% var = delta*delta
A=1;
H=1;
P=zeros(1,N);
P(1)=1;
K=zeros(1,N);
x_estimate = 0;
out(1) = 26.5;
for k=2:N
   x_estimate = A * out(k-1);% x(k|k-1) = A · x(k-1|k-1) + B · u(k)
   P(k) = A * P(k-1) * A' + Q;%P(k|k-1) = A · P(k-1|k-1) · AT + Q
   K(k) = P(k) * H' * inv(H * P(k) * H' +R);%K(k) = P(k|k-1) · HT · inv(H · P(k|k-1) · HT + R)
   out(k) = x_estimate + K(k) * (sample_tempture(k) - H * x_estimate);
   P(k) = (1 - K(k) * H) * P(k);
end
plot(out);
hold on;
plot(real_tempture);
plot(sample_tempture);
plot(tempture);
figure(2);
plot(out-real_tempture);
