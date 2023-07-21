clear all;
clc;
syms q1 q2 q1dot q2dot q1dotdot q2dotdot
h=[q1*q2,q1dot*q2dotdot;
    2*q1dot,q1;
    9*q1dotdot,q1;
    q2^4,q1*2;
    4*q2dot,q2*6;
     q2dotdot^3,q2*8];


cline1 = diff(h(1,:), q1);
cline2 = diff(h(2,:), q1dot);
cline3 = diff(h(3,:), q1dotdot);
cline4 = diff(h(4,:), q2);
cline5 = diff(h(5,:), q2dot);
cline6 = diff(h(6,:), q2dotdot);
hnew = [cline1;cline2;cline3;cline4;cline5;cline6];
