DH = [0, 0, 0, theta1;
      -pi/2, 0, 0, theta2;
      0, a2, d3, 0;
      0, a3, 0, 0];

% 机械臂的末端位置向量
end_effector_pos = [0; 0; 0; 1];

% 运动学变换矩阵（由基坐标系到末端坐标系）
T = eye(4);
for i = 1:size(DH, 1)
    theta = DH(i, 4);
    d = DH(i, 3);
    a = DH(i, 2);
    alpha = DH(i, 1);

    % 生成当前关节的旋转和平移变换矩阵
    A = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
         sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];

    % 累乘变换矩阵
    T = T * A;
end

% 机械臂末端位置向量（相对于基坐标系）
end_effector_pos_base = simplify(T * end_effector_pos);

% 提取末端位置向量中的坐标部分（忽略齐次坐标）
end_effector_pos_base = end_effector_pos_base(1:3);

% 计算雅可比矩阵
J = jacobian(end_effector_pos_base, [theta1, theta2, d3]);

% 显示雅可比矩阵
disp("Jacobian Matrix:");
disp(J);
