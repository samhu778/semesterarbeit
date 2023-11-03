function x_est_current = KF(x_est_previous, y_current)%1.Q值得大小 2.R和V要couple,v=通过randn生产一个满足方差是R的一个取样  ,3.H求不出严重
 %syms t
 delta_tk=0.1;
 x0 = x_est_previous;    
 p0=eye(6);                                
 F0 = kron([1,delta_tk,0.5*delta_tk^2;0,1,delta_tk;0,0,1], eye(2));                                   
 x10=F0*x0 ;%x10=6x1,F0=6x6
 R = 0.01*eye(12); %这里要改要和V一至
 v = sqrt(R) * randn(12, 1);
 h= y_current-v;
 %H1=H1_fun(h,x10);%h==12,x10==6行1列的向量，H1==12x6
 theta=[0;0;0];
 q = x10(1:2);
 dq = x10(3:4);
 ddq = x10(5:6);%?q dq ddq 是x10还是x0的部分
 [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta);
 [~, H1,~]= get_System_IMUMeasurement(q, dq, ddq, BodyParameter, FrameParameter, IMUParameter,v);
 Q=0.2*eye(6); 
 p10 = F0*p0*F0.'+Q ;   
 k1 = p10*H1.'*inv(H1*p10*H1.'+R) ;
 p11 = (eye(size(k1*H1))-k1*H1)*p10 ; 
 hx10 = FD2(x10);
 x11 = x10+k1*(y_current-hx10);
 x_est_current=x11;
end