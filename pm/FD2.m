function hx10 = FD2(x10)
    xblack = x10;
    QBlack = x10(1:4);
    QBlackdot = x10(5:8);
    QBlackdotdot = x10(9:12);

    q1 = QBlack(1) ;
    q2 = QBlack(2) ;
    q3 = QBlack(3) ;
    q4 = QBlack(4) ;

    q1dot = QBlackdot(1) ;
    q2dot = QBlackdot(2) ;
    q3dot = QBlackdot(3) ;
    q4dot = QBlackdot(4) ;


    q1dotdot = QBlackdotdot(1);
    q2dotdot = QBlackdotdot(2);
    q3dotdot = QBlackdotdot(3);
    q4dotdot = QBlackdotdot(4);

%%
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
    hx10=hfunction;

end