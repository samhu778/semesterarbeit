clear all; 
clc;
syms q1(t) q2(t) ;% quetion q p are about the time ,if i donot use ayms
%how to show diff?
q1 = pi/4*t*t ;
q2 = pi/4*t*t ;
q1dot = diff(q1,'t') ;
q2dot = diff(q2,'t') ;
q1dotdot = diff(q1dot,'t');
q2dotdot = diff(q2dot,'t');
thera = 1 ;

T01 = [cos(q1),-sin(q1),0,0;sin(q1),cos(q1),0,0;0,0,1,0;0,0,0,1]; 
R01 = [cos(q1),-sin(q1),0;sin(q1),cos(q1),0;0,0,1]; 
R10 = R01.';
P01 = [0,0,1].';

T12 = [cos(q2),-sin(q2),0,thera;sin(q2),cos(q2),0,0;0,0,1,0;0,0,0,1];
R12 = [cos(q2),-sin(q2),0;sin(q2),cos(q2),0;0,0,1];
R21 = R12.';
P12 = [thera,0,0].';


omega11 = [0,0,q1dot].' ;
omega22 = R21*omega11+[0,0,q2dot].' ;
velocity11 = 0 ;
velocity22 = R21*(velocity11 + cross(omega11, P12));


omega11dot = [0,0,q1dotdot].' ;
omega22dot = R21*omega11dot+cross(R21*omega11,[0,0,q2dot].')+[0,0,q2dotdot].';
velocity11dot  =  0;
velocity22dot = R21*(cross(omega11dot,P12)+cross(omega11,cross(omega11,P12))+0);


% make up h 

h1= [velocity11dot; omega11];

h2= [velocity22dot; omega22];

