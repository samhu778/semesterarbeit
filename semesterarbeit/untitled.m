clear all ;
clc ;
syms t x q1 q1dot q1dotdot q2 q2dot q2dotdot p 
delta_tk = 0.01;
for i = 1:2            
    x = [q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;q2;q2dot;q2dotdot;q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;q1;q1dot;q1dotdot];%(24x1)
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
    R = Q;        %（24x24）

    FK = kron([1,delta_tk,0.5*delta_tk^2;0,1,delta_tk;0,0,1], eye(8)) ; %FK(24x24)
    %xhat = FK * x ;    
    f = FK*x;    %f=xhat(24x1)
    F = jacobian(f, [x]); % F (24x24)
    phat = F*p*F.'+Q ;  %Phat（24x24） 

    h= [q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;
        q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;
        q1;q1dot;q1dotdot;q2;q2dot;q2dotdot;
        q1;q1dot;q1dotdot;q2;q2dot;q2dotdot];


    xhat = x ;%测试用(24x1)

    H = jacobian(h, [xhat]); %H（24x24）

    k = phat*H.'*inv(H*phat*H.'+R) ;

    p = (eye(size(k*H))-k*H)*phat ;
    %h是forward part 的
    cline1 = diff(h(1,:), q1);
    cline2 = diff(h(2,:), q1dot);
    cline3 = diff(h(3,:), q1dotdot);
    cline4 = diff(h(4,:), q2);
    cline5 = diff(h(5,:), q2dot);
    cline6 = diff(h(6,:), q2dotdot);
    cline7 = diff(h(7,:), q1);
    cline8 = diff(h(8,:), q1dot);
    cline9 = diff(h(9,:), q1dotdot);
    cline10 = diff(h(10,:), q2);
    cline11 = diff(h(11,:), q2dot);
    cline12 = diff(h(12,:), q2dotdot);
    cline13 = diff(h(13,:), q1);
    cline14 = diff(h(14,:), q1dot);
    cline15 = diff(h(15,:), q1dotdot);
    cline16 = diff(h(16,:), q2);
    cline17 = diff(h(17,:), q2dot);
    cline18 = diff(h(18,:), q2dotdot);
    cline19 = diff(h(19,:), q1);
    cline20 = diff(h(20,:), q1dot);
    cline21 = diff(h(21,:), q1dotdot);
    cline22 = diff(h(22,:), q2);
    cline23 = diff(h(23,:), q2dot);
    cline24 = diff(h(24,:), q2dotdot);



    hxat= [cline1,cline2,cline3,cline4,cline5,cline6,cline7,cline8,cline9,cline10,cline11,cline12,cline13,cline14,cline15,cline16,cline17,cline18,cline19,cline20,cline21,cline22,cline23,cline24]*xhat; 
    y=eye(24);%测试
    x = xhat+k*(y-hxat) ;
    %xlswrite('',x) ;
end
