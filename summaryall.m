clc;
clear;
syms tvalue t

x_est_0=[1;1;1;1;1;1;1;1;1;1;1;1];
x_est_current = x_est_0;% 在overleaf上写一个text 一个kalmanfilter的流程，一边写一边转成code，一一对应
% for true

    x_est_previous = x_est_current;
    % 1 obtain measurement
%     if use_real_sensor
%         y_current = load_IMU;%代入计算
%     elseif use_simulated_sensor
        y_current = FD(t);%暂时没有imu 用这个模拟imu
%     end
    % 2 Estimation x
    x_est_current = KF(x_est_previous, y_current);
% end




function x_est_current = KF(x_est_previous, y_current)
 syms t
 delta_tk=0.1; 

 %initial value self-defined 
 %initial equation %x1=F0*X0+W0
                   %y1=h(x1)+v0
 x0 = x_est_previous;    
 p0=eye(12);

%prediction 
     %accoding to essay,i can get F0  
     %This step is also linearzation of the initial equation
     F0 = kron([1,delta_tk,0.5*delta_tk^2;0,1,delta_tk;0,0,1], eye(4));
    
     v=[1;1;1;1;1;1;1;1;1;1;1;1];
     h= y_current-v;
     f0=F0*x0;
     x10=f0 ;
       for i=1:1:12  % H1 = jacobian(h, [x10]); %H（12x12）
            for j=1:1:12
             vh(i) = diff(h(i,:), t);
             vx10(j) = diff(x10(j,:), t);
             if vx10(j)==0
                        H1(i,j)=0;
                    else
                        H1(i,j)=vh(i)/vx10(j);
                    end
            end
       end
       Q=[1;1;1;1;1;1;1;1;1;1;1;1]; 
       p10 = F0*p0*F0.'+Q ;   %P10（12x12) = new p 

%correction
    R = eye(12);
    k1 = p10*H1.'*inv(H1*p10*H1.'+R) ;
    p11 = (eye(size(k1*H1))-k1*H1)*p10 ; 
    for i=1:1:12   % jacobian(h, [x10]) ;
             for j=1:1:12
              vh1(i) = diff(h(i,:), t);
              vx10(j) = diff(x10(j,:), t);
                 if vx10(j)==0
                            b(i,j)=0;
                        else
                             b(i,j)=vh1(i)/vx10(j);
                  end
            end
    end
    hx10=b*x10;%hxat = jacobian(h, [x]) * x10;
    x11 = x10+k1*(y_current-hx10)
    x_est_current=x11;
end


function y = FD(t)
% x = x(t);%自己定义一个实际的jonit一个状态
% h(x)     %速度加速度的向量
% y = h(x) + v;%观测值y
    syms t 
    q1 = sin(t) ;
    q2 = sin(2*t) ;
    q3 = sin(3*t) ;
    q4 = sin(4*t) ;
    qimu1 = sin(5*t);
    qimu2 = sin(6*t);

    g=[0;0;9.81];
    
    q1dot = diff(q1,'t') ;
    q2dot = diff(q2,'t') ;
    q3dot = diff(q3,'t') ;
    q4dot = diff(q4,'t') ;
    qimu1dot = diff(qimu1,'t')  ;
    qimu2dot = diff(qimu2,'t')  ;
   

    q1dotdot = diff(q1dot,'t');
    q2dotdot = diff(q2dot,'t');
    q3dotdot = diff(q3dot,'t');
    q4dotdot = diff(q4dot,'t');
    qimu1dotdot = diff(qimu1dot,'t')  ;
    qimu2dotdot = diff(qimu2dot,'t')  ;
    
    distance_oneplustwo = 1.7 ;
    distance_three = 0.5;
    distance_four = 0.5 ;
    distence2imu1 = 0.25;
    distance3imu2 = 0.25 ; 
    
  %01
    T01 = [cos(q1),-sin(q1),0,0;
          sin(q1),cos(q1),0,0;
          0,0,1,distance_oneplustwo;
          0,0,0,1]; 
    R01 = [cos(q1),-sin(q1),0;
           sin(q1),cos(q1),0;
           0,0,1]; 
    R10 = R01.';
    P01 = [0,0,distance_oneplustwo].';
    
                       
                        
  %12
    T12 = [cos(q2),-sin(q2),0,0;
          0,0,-1,0;
          sin(q2),cos(q2),0,0;
          0,0,0,1];
    R12 = [cos(q2),-sin(q2),0;
          0,0,-1;
          sin(q2),cos(q2),0];
    R21 = R12.';
    P12 = [0,0,0].';
                        
  %23
    T23 = [cos(q3),-sin(q3),0,distance_three;
           sin(q3),cos(q3),0,0;
          0,0,1,0;
          0,0,0,1];
    R23 = [cos(q3),-sin(q3),0;
           sin(q3),cos(q3),0;
          0,0,1;];
    R32 = R23.';
    P23 = [distance_three,0,0].';

                        T2imu1 = [1,0,0,distence2imu1;
                                  0,1,0,0;
                                  0,0,1,0;
                                  0,0,0,1];
                        R2imu1 = [1,0,0;
                                  0,1,0;
                                  0,0,1;];
                        Rimu12 = R2imu1.';
                        P2imu1 = [distence2imu1,0,0].';   

                        
%34                        
    T34 = [cos(q4),-sin(q4),0,distance_four;
           sin(q4),cos(q4),0,0;
          0,0,1,0;
          0,0,0,1];
    R34 = [cos(q4),-sin(q4),0;
           sin(q4),cos(q4),0;
          0,0,1;];
    R43 = R34.';
    P34 = [distance_four,0,0].';   

                       T3imu2 = [1,0,0,distance3imu2;
                                  0,1,0,0;
                                  0,0,1,0;
                                  0,0,0,1];
                        R3imu2 = [1,0,0;
                                  0,1,0;
                                  0,0,1];
                        Rimu23 = R3imu2.';
                        P3imu2 = [distance3imu2,0,0].';
    

    %计算part%
    % 1 -4
    
    omega11 = [0,0,q1dot].' ;
    omega22 = R21*omega11+[0,0,q2dot].' ;
    omega33 = R32*omega22+[0,0,q3dot].' ;
    omega44 = R43*omega33+[0,0,q4dot].' ;
    velocity11 = 0 ;
    velocity22 = R21*(velocity11 + cross(omega11, P12));
    velocity33 = R32*(velocity22 + cross(omega22, P23));
    velocity44 = R43*(velocity33 + cross(omega33, P34));


    omega11dot = [0,0,q1dotdot].' ;
    omega22dot = R21*omega11dot+cross(R21*omega11,[0,0,q2dot].')+[0,0,q2dotdot].';
    omega33dot = R32*omega22dot+cross(R32*omega22,[0,0,q3dot].')+[0,0,q3dotdot].';
    omega44dot = R43*omega33dot+cross(R43*omega33,[0,0,q4dot].')+[0,0,q4dotdot].';

    velocity11dot  =  g;
    velocity22dot = R21*(cross(omega11dot,P12)+cross(omega11,cross(omega11,P12))+velocity11dot);  
    velocity33dot = R32*(cross(omega22dot,P23)+cross(omega22,cross(omega22,P23))+velocity22dot);
    velocity44dot = R43*(cross(omega33dot,P34)+cross(omega33,cross(omega33,P34))+velocity33dot);%每个都加g?
    
    %1-imu1
    omegaimu1imu1 = Rimu12*omega22+[0,0,0].' ;
    velocityimu1imu1 = Rimu12*(velocity22 + cross(omega22,  P2imu1));
    omegaimu1imu1dot = Rimu12*omega22dot+cross(Rimu12*omega22,[0,0,0].')+[0,0,0].';
    velocityimu1imu1dot = Rimu12*(cross(omega22dot,P2imu1)+cross(omega22,cross(omega22,P2imu1))+velocity22dot);
    
    %2-imu2
    omegaimu2imu2 = Rimu23*omega33+[0,0,0].' ;
    velocityimu2imu2 = Rimu23*(velocity33 + cross(omega33,  P3imu2));
    omegaimu2imu2dot = Rimu23*omega22dot+cross(Rimu23*omega33,[0,0,0].')+[0,0,0].';
    velocityimu2imu2dot = Rimu23*(cross(omega33dot,P3imu2)+cross(omega33,cross(omega33,P3imu2))+velocity33dot);
    
    
    
    % make up h 
    
    h1= [velocityimu1imu1dot; omegaimu1imu1];
    
    h2= [velocityimu2imu2dot; omegaimu2imu2];

    T02 = T01*T12;
    T03 = T01*T12*T23 ;
    T04 = T01*T12*T23*T34 ;            
    
    lastline2 = T02(:, end);
    Placejoint2 = lastline2(1:3,:);
    lastline3 = T03(:, end);
    Placejoint3 = lastline3(1:3,:);
    lastline4 = T04(:, end);
    Placejoint4 = lastline4(1:3,:);
  
    
    hfunction=[h1;h2]%(12x1);
    v=[1;1;1;1;1;1;1;1;1;1;1;1];%according to essay V:measurement noise(mean=0,Gaussian distribution)
    y=hfunction+v;

%     yt2=subs(Placejoint2, t, tvalue);  %替换t为自变量tvalue
%     yt3=subs(Placejoint3, t, tvalue);
%     yt4=subs(Placejoint4, t, tvalue);

end