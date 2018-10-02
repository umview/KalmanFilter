%%%%%%%%%%%%%
% 卡尔曼滤波用于自由落体运动目标跟踪问题
%%%%%%%%%%%%%%
N=1000; %仿真时间，时间序列总数
%噪声
Q=[0,0;0,0];%过程噪声方差为0，即下落过程忽略空气阻力
R=1; %观测噪声方差
W=sqrt(Q)*randn(2,N);%既然Q为0，即W=0
V=sqrt(R)*randn(1,N);%测量噪声V(k)
%系数矩阵
A=[1,1;0,1];%状态转移矩阵
B=[0.5;1];%控制量
U=-1;
H=[1,0];%观测矩阵
%初始化
X=zeros(2,N);%物体真实状态
X(:,1)=[95;1];%初始位移和速度
P0=[10,0;0,1];%初始误差
Z=zeros(1,N);
Z(1)=H*X(:,1);%初始观测值
Xkf=zeros(2,N);%卡尔曼估计状态初始化
Xkf(:,1)=X(:,1);
err_P=zeros(N,2);
err_P(1,1)=P0(1,1);
err_P(1,2)=P0(2,2);
I=eye(2); %二维系统
%%%%%%%%%%%%%
for k=2:N
    %物体下落，受状态方程的驱动
    X(:,k)=A*X(:,k-1)+B*U+W(k);
    %位移传感器对目标进行观测
    Z(k)=H*X(:,k)+V(k);
    %卡尔曼滤波
    X_pre=A*Xkf(:,k-1)+B*U;%状态预测 
    P_pre=A*P0*A'+Q;%协方差预测
    Kg=P_pre*H'*inv(H*P_pre*H'+R);%计算卡尔曼增益
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);%状态更新
    P0=(I-Kg*H)*P_pre;%方差更新
    %误差均方值
    err_P(k,1)=P0(1,1);
    err_P(k,2)=P0(2,2);
end
%误差计算
messure_err_x=zeros(1,N);%位移的测量误差
kalman_err_x=zeros(1,N);%卡尔曼估计的位移与真实位移之间的偏差
kalman_err_v=zeros(1,N);%卡尔曼估计的速度与真实速度之间的偏差
for k=1:N
    messure_err_x(k)=Z(k)-X(1,k);
    kalman_err_x(k)=Xkf(1,k)-X(1,k);
    kalman_err_v(k)=Xkf(2,k)-X(2,k);
end
%%%%%%%%%%%%%%%
%画图输出
%噪声图
figure
plot(V);
title('messure noise')
%位置偏差
figure
hold on,box on;
plot(messure_err_x,'-r.');%测量的位移误差
plot(kalman_err_x,'-g.');%卡尔曼估计位置误差
legend('测量误差','kalman估计误差')
figureplot(kalman_err_v);
title('速度误差')
figure
plot(err_P(:,1));
title('位移误差均方值')
figure
plot(err_P(:,1));
title('速度误差均方值')
%%%%%%%%%%%%%%%%%%%%%%%