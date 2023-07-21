clear all
clc;


q = [pi/4,pi/4]; %initial the relativ rotation angle of shoulder and elbow by pi/4(guess value)
qdot = [pi/4,pi/4]; %initial the relativ rotation speed of shoulder and elbow by pi/4(guess value)
qdotdot = [pi/4,pi/4]; %initial the relativ rotation acceleration of shoulder and elbow by pi/4(guess value)
x = [q.', qdot.',qdotdot.'] ;
x = x.' ;
phi = [1,2,3];  % fai is rotation vector, 
                %%%%%%%%%%%%%%%%%%%%%%
                % Question 
                % I do not know the ratation vector,phi
                % and in this
                % part,there are a lot of r,that contribute to theta,so i
                % just use r1 for joint between 1.2 ,or use many r for many
                % joint ? 
                % I think it is just r for 1.2,but not sure 
theta = norm (phi);
phix =[0,-phi(1,3),phi(1,2);phi(1,3),0,-phi(1,2);-phi(1,2),phi(1,1),0] ;
R = eye(3)+sin(theta)/theta*phix+(1-cos(theta)/theta/theta*phix*phix);

omega = [1,2,3];
                %%%%%%%%%%%%%%%%%%%%%%
                % Question 
                % IMU will directly give the value of omega ?
omegax= [0,-omega(1,3),omega(1,2);omega(1,3),0,-omega(1,2);-omega(1,2),omega(1,1),0] ;
r = [t,2t,3t]  ; 
r = r+R*r;
rdot =diff(r,'t')+R*omegax*r ;
rdotdot = diff(rdot,'t')+R*omegax*omegax*r+R*diff(omega,'t')*r ;


