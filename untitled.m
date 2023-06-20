clear all ;
clc ;
syms x t y 
y = eye(9) ;   %加速度速度矩阵 Question:y是接入LMU的a,w，是一个3x2的矩阵吗?
h = eye(9) ;   %h是x的函数

v = eye(9) ;   %v是noise 所以应该是什么样的？
R = eye(9);    %R是和V相关，但是自己假设
 


x = eye(9) ;   %x是joint[位置，速度，加速度]9X9矩阵
f = eye(9) ;   %f是x的函数
p = eye(9) ;   %初始化p 
H = eye(9) ;   %初始化H
F = eye(9) ;   %初始化F
k = eye(9) ;   %初始化k

w = eye(9) ;   %w是noise
Q = eye(9) ;   %Q是和w相关，自己假设

for i = 1:100            %question 本实验应该是接收到一个y信号算一次卡曼循环
                         %感觉这样子写循环应该不太对
    y(i) = xlsread('E:\semesterarbeit\lum.xlsx') ; %y读取文件，Question LMU输出的格式是xls吗？
    h(i-1) = y(i)-v ;                      %准备h
    A = [1,t(i)-t(i-1),0.5*(t(i)-t(i-1))^2;0,1,(t(i)-t(i-1));0,0,1] ;%这个delta tk 是1s 吗？
    B = eye(size(A)) ;
    f(i-1) = kron(A, B)*x(i-1) ;           %准备f，开始i=0 x0=预设量

    xhat(i-1) = f(i-1) ;                   % xhat 是predition value
    F(i) = diff(f,'x(i-1)') ;
    phat(i-1) = F(i)*p(i-1)*F(i).' + Q ;   % phat 是predition value
   
    H(i) = diff(h,'xhat(i-1)') ;
    k(i) = phat(i-1)*H(i).'*inv(H(i)*phat(i-1)*H(i).'+R) ;
    p(i) = (eye(size(k(i)*H(i)))-k(i)*H(i))*phat(i-1) ;
    x(i) = xhat(i-1)+k(i)*(y(i)-h(i-1)) ;
    xlswrite('',x(i)) ;
end
