% Kalman filter example of temperature measurement in Matlab implementation of Kalman filter algorithm.
% 房间当前温度真实值为24度，认为下一时刻与当前时刻温度相同，误差为0.02度（即认为连续的两个时刻最多变化0.02度）。
% 温度计的测量误差为0.5度。
% 开始时，房间温度的估计为23.5度，误差为1度。
close all;
clear all;
% intial parameters
% 计算连续n_iter个时刻
n_iter = 1000;
% size of array. n_iter行，1列
sz = [n_iter, 1];
% 温度的真实值
x = 24;
% 过程方差，反应连续两个时刻温度方差。更改查看效果
Q = 4e-4;
% 测量方差，反应温度计的测量精度。更改查看效果
R = 0.25;
% z是温度计的测量结果，在真实值的基础上加上了方差为0.25的高斯噪声。
z = x + sqrt(R)*randn(sz);
% 对数组进行初始化
% 对温度的后验估计。即在k时刻，结合温度计当前测量值与k-1时刻先验估计，得到的最终估计值
xhat = zeros(sz); 
% 后验估计的方差
P = zeros(sz); 
% 温度的先验估计。即在k-1时刻，对k时刻温度做出的估计
xhatminus = zeros(sz);
% 先验估计的方差
Pminus = zeros(sz);
% 卡尔曼增益，反应了温度计测量结果与过程模型（即当前时刻与下一时刻温度相同这一模型）的可信程度
K = zeros(sz); 
% intial guesses
xhat(1) = 0.5; %温度初始估计值为23.5度
P(1) =1; % 误差方差为1
 
for k = 2:n_iter
    % 时间更新（预测）
    % 用上一时刻的最优估计值来作为对当前时刻的温度的预测
    xhatminus(k) = xhat(k-1);
    % 预测的方差为上一时刻温度最优估计值的方差与过程方差之和
    Pminus(k) = P(k-1)+Q;
    % 测量更新（校正）
    % 计算卡尔曼增益
    K(k) = Pminus(k)/( Pminus(k)+R );
    % 结合当前时刻温度计的测量值，对上一时刻的预测进行校正，得到校正后的最优估计。该估计具有最小均方差
    xhat(k) = xhatminus(k)+K(k)*(z(k)-xhatminus(k));
    % 计算最终估计值的方差
    P(k) = (1-K(k))*Pminus(k);
end
 
FontSize = 14;
LineWidth = 3;
figure();
plot(z,'k+'); %画出温度计的测量值
hold on;
plot(xhat,'b-','LineWidth',LineWidth) %画出最优估计值
hold on;
plot(x*ones(sz),'g-','LineWidth',LineWidth); %画出真实值
legend('温度计的测量结果', '后验估计', '真实值');
xl = xlabel('时间(分钟)');
yl = ylabel('温度');
set(xl,'fontsize',FontSize);
set(yl,'fontsize',FontSize);
hold off;
set(gca,'FontSize',FontSize);
 
figure();
valid_iter = 2:n_iter; % Pminus not valid at step 1
% 画出最优估计值的方差
plot(valid_iter,P(valid_iter),'LineWidth',LineWidth);
legend('后验估计的误差估计');
xl = xlabel('时间(分钟)');
yl = ylabel('℃^2');
set(xl,'fontsize',FontSize);
set(yl,'fontsize',FontSize);
set(gca,'FontSize',FontSize);