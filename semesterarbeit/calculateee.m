clear all;
clc;
n = 2;
m = 1;
TotalStep = 100;
A = diag(rand(1,n)); %创建一个对角矩阵 A，其中对角线上的元素是一个包含n 个随机数的行向量。
eig(A)
B = 1*rand(n,n);%创建一个大小为 n×n (2x2)的随机矩阵 B，其中每个元素都是从 0 到 1 之间均匀分布的随机数。
C = rand(m,n);%创建一个大小为 m×n (1x2)的随机矩阵 C，其中每个元素都是从 0 到 1 之间均匀分布的随机数。

w_set = 0.1*rand(n,TotalStep);%w-set 是nx100的随机矩阵，其中每个元素都是从 0 到 0.1 之间均匀分布的随机数。
Q = cov(w_set');   %Q是w-set转置的协方差：矩阵的协方差矩阵是一个 m×m 的矩阵，用于描述矩阵中各个维度之间的线性关系和各个维度的方差
W = eye(n); %w 是nxn对角矩阵

v_set = 0.1*rand(n,TotalStep);%v-set 是nx100的随机矩阵，其中每个元素都是从 0 到 0.1 之间均匀分布的随机数。
R = cov(v_set');%同上
V = rand(m,n);%同上


x0 = 10*(rand(n,1)+1);% 创建一个大小为 n×1 的随机列向量 x0，其中每个元素是从 10 到 20 之间均匀分布的随机数。
xk_1_est = x0;
xk_1_true = x0;
xk_1_ideal = x0;

Pk_1 = 0*eye(n);%nxn 的0矩阵

Pk_set = nan(n,n,TotalStep);% 创建一个大小为 n×n×TotalStep 的三维数组 Pk_set，用于存储 n×n 的矩阵，且共有 TotalStep 个矩阵。初始时，所有的元素都被设置为 NaN。
Pk_set(:,:,1) = Pk_1;%是对 Pk_set 三维数组中的第一个矩阵进行索引操作，冒号表示取所有的行和列，1 表示第一个矩阵。

xk_est_set = nan(n,TotalStep);%创建一个大小为 n×TotalStep 的二维数组 xk_est_set，用于存储 n 维的向量，且共有 TotalStep 个向量。初始时，所有的元素都被设置为 NaN。
xk_est_set(:,1) = xk_1_est; %xk_1_est = x0赋值给xk_est_set的第一列

xk_true_set = nan(n,TotalStep);%创建一个大小为 n×TotalStep 的二维数组 xk_true_set，用于存储 n 维的向量，且共有 TotalStep 个向量。初始时，所有的元素都被设置为 NaN。
xk_true_set(:,1) = xk_1_true; %xk_1_true = x0;赋值给xk_est_set的第一列

xk_ideal_set = nan(n,TotalStep);%同上
xk_ideal_set(:,1) = xk_1_ideal;

for k = 1:TotalStep-1
	uk_1 = 1*rand(n,1)+0.0001*k^2; %生成一个大小为 n×1 的随机列向量 uk_1，其中每个元素都是从 1 到 1+0.0001*k^2 之间均匀分布的随机数。

	xk_ideal = A * xk_1_ideal + B * uk_1; %x=Ax+Bu : A(2x2)  x(2x1)=(2x1);B(2x2) u(2x1)=(2x1)====(2x1)
	yk_ideal = C * xk_ideal;              %y=Cx: C(1x2) x(2x1)=(1x1)

	xk_true = A * xk_1_true + B * uk_1 + W * w_set(:,k);%A(2x2) xk_1_true(2x1)+ B(2x2) W ()* w_set()
end 