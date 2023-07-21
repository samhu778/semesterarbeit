clear all; 
clc;
syms q1(t) q2(t) ;% 这样的方法，是知道0处的速度加速度，q1,q2的一个运动函数才能用
%how to show diff?
q1 = pi/4*t*t ;
qimu1 = pi/4*t*t ;
q2 = pi/4*t*t ;
qimu2 = pi/4*t*t;
g=[0;0;9.81];

q1dot = diff(q1,'t') ;
q2dot = diff(q2,'t') ;
qimu1dot = diff(qimu1,'t')  ;
qimu2dot = diff(qimu2,'t')  ;

q1dotdot = diff(q1dot,'t');
q2dotdot = diff(q2dot,'t');
qimu1dotdot = diff(qimu1dot,'t')  ;
qimu2dotdot = diff(qimu2dot,'t')  ;

distance12 = 1 ;
distence1imu1 = 0.5;
distance2imu2 = 0.5 ;


T01 = [cos(q1),-sin(q1),0,0;sin(q1),cos(q1),0,0;0,0,1,0;0,0,0,1]; 
R01 = [cos(q1),(q1),0;sin(q1),cos(q1),0;0,0,1]; 
R10 = R01.';
P01 = [0,0,1].';

T1imu1 = [cos(qimu1),-sin(qimu1),0,distence1imu1;sin(qimu1),cos(qimu1),0,0;0,0,1,0;0,0,0,1];
R1imu1 = [cos(qimu1),-sin(qimu1),0;sin(qimu1),cos(qimu1),0;0,0,1];
Rimu11 = R1imu1.';
P1imu1 = [distence1imu1,0,0].';


T12 = [cos(q2),-sin(q2),0,distance12;sin(q2),cos(q2),0,0;0,0,1,0;0,0,0,1];
R12 = [cos(q2),-sin(q2),0;sin(q2),cos(q2),0;0,0,1];
R21 = R12.';
P12 = [distance12,0,0].';

T2imu2 = [cos(qimu2),-sin(qimu2),0,distance2imu2;sin(qimu2),cos(qimu2),0,0;0,0,1,0;0,0,0,1];
R2imu2 = [cos(qimu2),-sin(qimu2),0;sin(qimu2),cos(qimu2),0;0,0,1];
Rimu22 = R2imu2.';
P2imu2 = [distance2imu2,0,0].';

%1 -2
omega11 = [0,0,q1dot].' ;
omega22 = R21*omega11+[0,0,q2dot].' ;
velocity11 = 0 ;
velocity22 = R21*(velocity11 + cross(omega11, P12));

omega11dot = [0,0,q1dotdot].' ;
omega22dot = R21*omega11dot+cross(R21*omega11,[0,0,q2dot].')+[0,0,q2dotdot].';
velocity11dot  =  g;
velocity22dot = R21*(cross(omega11dot,P12)+cross(omega11,cross(omega11,P12))+g+0);

%1-imu1
omegaimu1imu1 = Rimu11*omega11+[0,0,q2dot].' ;
velocityimu1imu1 = Rimu11*(velocity11 + cross(omega11,  P1imu1));
omegaimu1imu1dot = Rimu11*omega11dot+cross(Rimu11*omega11,[0,0,qimu1dot].')+[0,0,qimu1dotdot].';
velocityimu1imu1dot = Rimu11*(cross(omega11dot,P1imu1)+cross(omega11,cross(omega11,P1imu1)+g)+0);

%2-imu2
omegaimu2imu2 = R2imu2*omega22+[0,0,qimu2dot].' ;
velocityimu2imu2 = Rimu22*(velocity22 + cross(omega22,  P2imu2));
omegaimu2imu2dot = Rimu22*omega22dot+cross(Rimu22*omega22,[0,0,qimu2dot].')+[0,0,qimu2dotdot].';
velocityimu2imu2dot = Rimu22*(cross(omega22dot,P2imu2)+cross(omega22,cross(omega22,P2imu2)+g)+0);



% make up h 

h1= [velocityimu1imu1dot; omegaimu1imu1];

h2= [velocityimu2imu2dot; omegaimu2imu2];

h3= [velocity11dot; omega11];

h4= [velocity22dot; omega22];

h=[h1;h2];
