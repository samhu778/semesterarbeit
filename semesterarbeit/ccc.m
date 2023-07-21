clear all ;
clc ;
syms t x q1 q1dot q1dotdot q2 q2dot q2dotdot
delta_tk = 0.01;            
    x = [q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;];
    Qomega = [delta_tk^5/20,delta_tk^4/8,delta_tk^3/6;delta_tk^4/8,delta_tk^3/3,delta_tk^2/2;delta_tk^3/6,delta_tk^2/2,delta_tk^5] ;
    Qrho = [1,0,0,0,0,0,0,0;
            0,1,0,0,0,0,0,0;
            0,0,1,0,0,0,0,0;
            0,0,0,1,0,0,0,0;
            0,0,0,0,1,0,0,0;
            0,0,0,0,0,1,0,0;
            0,0,0,0,0,0,1,0;
            0,0,0,0,0,0,0,1;];     
    Q = kron(Qomega, Qrho);
    R = Q;        

    FK = kron([1,delta_tk,0.5*delta_tk^2;0,1,delta_tk;0,0,1], eye(2)) ;
    xhat = FK * x ;       %x怎么输入或者设置，%y = xlsread('E:\semesterarbeit\lum.xlsx') ; %y how to get date？
    f = FK*x;    %f(6x1
    F = jacobian(f, [x]);%F(6x6)  F*F=(6x6)
    phat = F*p*F.'+Q ;   % phat is predition value(Q=24X24)