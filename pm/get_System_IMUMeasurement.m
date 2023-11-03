

function [yk, Hk, Vk]= get_System_IMUMeasurement(q, dq, ddq, BodyParameter, FrameParameter, IMUParameter, vk)
    n = length(q) ;    
    IMU_Quantity = length(IMUParameter);%get_SystemParameter=BodyParameterClass(BodyConnectionParameterClass)+IMUParameterClass+FrameParameterClass(JointParameterClass)
    m = 6 * IMU_Quantity ;%m这里是12
    Frame = FrameClass(n, FrameParameter);%FrameClass=FixedRotation_ForwardKinematics(get_R)
    Frame = Frame.set_Joint(zeros([n, 1]));
    Body = {}; 
    for i = 1:numel(BodyParameter)%get_SystemParameter
        BodyElementParameter = BodyParameter(i); 
        bodyObject = BodyClass(q, dq, BodyElementParameter, Frame, Body);
         bodyObject =bodyObject.set_Joint(dq, BodyElementParameter);
        Body{end+1} = bodyObject;
    end

    yk = [];%yk是12*1,为啥显示的是18？？？？？？？？？？？？？
    
    Hk = [];
    gravity_amplitude = 9.8 * ones([IMU_Quantity, 1]);
    
   for IMU_Nr=1:1:IMU_Quantity
   
        IMUElementParameter=IMUParameter(IMU_Nr) ;
        BodyNr=IMUElementParameter.Body;
        [r,R,Tr,dTr,Tphi,dTphi,dddrdtdt_dq,dddrdtdt_ddq]= set_IMU_Joint(dq,IMUElementParameter,Body{BodyNr});
        omega=Tphi*dq;
        ddrdtdt=Tr*ddq+dTr*dq;
        g=[0,0,gravity_amplitude(IMU_Nr,1)].';
        trans_R=R';
        Acceleration=trans_R*(ddrdtdt+g);
        AngularVelocity=omega;
        yk_i=[Acceleration;AngularVelocity];
        Hk_i_a=horzcat(trans_R*dddrdtdt_dq+trans_R*skew(g)*R*Tphi-skew(omega)*trans_R*dTr, ...
            trans_R*dddrdtdt_ddq-skew(omega)*trans_R*Tr, ...
            trans_R*Tr);
        Hk_i_w=horzcat(dTphi,Tphi,zeros(3, n));
        Hk_i=vertcat(Hk_i_a, Hk_i_w);
        yk=vertcat(yk, yk_i);
        Hk=vertcat(Hk, Hk_i);
        
    end 
     yk = yk + vk;%%%%%yk is true,vk not same
     Vk = eye(m);
      
    
end

