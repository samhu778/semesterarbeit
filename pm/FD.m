function [y,xblack] = FD(t)
% x = x(t);%自己定义一个实际的jonit一个状态
% h(x)     %速度加速度的向量
% y = h(x) + v;%观测值y
    %%%%%%%%%%%1.t没用到，删了FD(t)之后报错，改的话直接手算
%     q1 = sin(t) ;
%     q2 = sin(2*t) ;
%     q3 = sin(3*t) ;
%     q4 = sin(4*t) ;
%     qimu1 = sin(5*t);
%     qimu2 = sin(6*t);
%  
%     q1dot = cos(t) ;
%     q2dot = 2*cos(2*t) ;
%     q3dot = 3*cos(3*t) ;
%     q4dot = 4*cos(4*t) ;
%     qimu1dot = 5*cos(5*t)  ;
%     qimu2dot = 6*cos(6*t)  ;
%    
%     q1dotdot = -sin(t);
%     q2dotdot = -4*sin(2*t);
%     q3dotdot = -9*sin(3*t);
%     q4dotdot = -16*sin(4*t);
%     qimu1dotdot = -25*sin(5*t)  ;
%     qimu2dotdot = -36*sin(6*t)  ;
%     QBlack=[q1;q2;q3;q4];
%     QBlackdot=[q1dot;q2dot;q3dot;q4dot];
%     QBlackdotdot=[q1dotdot;q2dotdot;q3dotdot;q4dotdot];
%     xblack=[QBlack;QBlackdot;QBlackdotdot];

    q0 = deg2rad([0;0]);
    qe = deg2rad([45; 90]);
    t=0;
    t_start=1;
    t_end=1000; 
    [q,dq,ddq] = Polynomial_FirstSecondOrder0_Function(t,t_start,t_end*0.8,q0,qe);

    QBlack=q; %q 2x1
    q1=QBlack(1);
    q2=QBlack(2);
    
    QBlackdot=dq;%dq 2x1
    QBlackdotdot=ddq;%ddq 2x1
    xblack=[QBlack;QBlackdot;QBlackdotdot];%xblack 6x1


    g=[0;0;9.81];

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
  
    
    hfunction=[h1;h2];%(12x1);
    v=0*[1;1;1;1;1;1;1;1;1;1;1;1];%according to essay V:measurement noise(mean=0,Gaussian distribution)
    y=hfunction+v;

    

%     yt2=subs(Placejoint2, t, tvalue);  %替换t为自变量tvalue
%     yt3=subs(Placejoint3, t, tvalue);
%     yt4=subs(Placejoint4, t, tvalue);

end