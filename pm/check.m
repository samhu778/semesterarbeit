clc;
clear;

q0 = deg2rad([0;0]);
qe = deg2rad([45; 90]);
dq0 = zeros(size(q0));
zero_matrix = zeros(size(q0));
x0 = cat(1, cat(1, q0, zero_matrix), zero_matrix);
t=0;
t_start=1;
t_end=1000; 
theta=[0;0;0];
% BodyNr=1;
% JointNr=2;
% RotationAxis='y';
% RotationAngle='2';

[q,dq,ddq] = Polynimial_FirstSecondOrder0_Function(t,t_start,t_end*0.8,q0,qe);
[BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta);
IMU_Quantity = length(IMUParameter); 
m = 6 * IMU_Quantity ;
vk=zeros([m, 1]);
[yk, Hk,Vk]= get_System_IMUMeasurement(q, dq, ddq, BodyParameter, FrameParameter, IMUParameter,vk);