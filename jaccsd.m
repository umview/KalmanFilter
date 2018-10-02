function [z, A] = jaccsd(fun, x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
%
z = fun(x);
% numel返回数组中元素个数。若是一幅图像，则numel(A)将给出它的像素数。
n = numel(x);
m = numel(z);
A = zeros(m,n);
h = n*eps;
for k = 1:n
    x1 = x;
    x1(k) = x1(k)+h*1i;
    % imag返回复数的虚部
    A(:,k) = imag(fun(x1))/h;
end
