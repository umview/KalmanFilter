% function: simulating the process of EKF
N = 50;           % 计算连续N个时刻
n = 3;             % 状态维度
q = 0.1;          % 过程标准差
r = 0.2;           % 测量标准差
% eye函数产生单位矩阵
Q = q^2*eye(n);   % 过程方差
R = r^2;               % 测量值的方差
 
%{
     FUNHANDLE = @FUNCTION_NAME returns a handle to the named function,
     FUNCTION_NAME. A function handle is a MATLAB value that provides a
     means of calling a function indirectly. 
%}
f = @(x)[x(2);x(3);0.05*x(1)*(x(2)+x(3))];  % 状态方程
h = @(x)[x(1);x(2);x(3)];                          % 测量方程
s = [0;0;1];                                            % 初始状态
 
% 初始化状态
x = s+q*randn(3,1); 
% eye返回单位矩阵
P = eye(n);
% 3行50列，一列代表一个数据
% 最优估计值
xV = zeros(n,N);
% 真实值
sV = zeros(n,N);
% 状态测量值
zV = zeros(n,N);
 
for k = 1:N
    z = h(s) + r * randn;
    % 实际状态
    sV(:,k) = s;
    % 状态测量值
    zV(:,k) = z;
    
    % 计算f的雅可比矩阵，其中x1对应黄金公式line2
    [x1,A] = jaccsd(f,x);
    % 过程方差预测，对应line3
    P = A*P*A'+Q;
    % 计算h的雅可比矩阵
    [z1,H] = jaccsd(h,x1);
    
    % 卡尔曼增益，对应line4
    % inv返回逆矩阵
    K = P*H'*inv(H*P*H'+R);
    % 状态EKF估计值，对应line5
    x = x1+K*(z-z1);
    % EKF方差，对应line6
    P = P-K*H*P;
    
    % save
    xV(:,k) = x;
    % update process 
    s = f(s) + q*randn(3,1);
end
for k = 1:3
    FontSize = 14;
    LineWidth = 1;
    
    figure();
    % 画出真实值
    plot(sV(k,:),'g-');
    hold on;
    
    % 画出最优估计值
    plot(xV(k,:),'b-','LineWidth',LineWidth);
    hold on;
    
    % 画出状态测量值
    plot(zV(k,:),'k+');
    hold on;
    
    legend('真实状态', 'EKF最优估计估计值','状态测量值');
    xl = xlabel('时间(分钟)');
    % 把数值转换成字符串， 转换后可以使用fprintf或disp函数进行输出。
    t = ['状态 ',num2str(k)] ;
    yl = ylabel(t);
    set(xl,'fontsize',FontSize);
    set(yl,'fontsize',FontSize);
    hold off;
    set(gca,'FontSize',FontSize);
end
