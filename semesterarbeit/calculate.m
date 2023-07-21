clear all 
clc;

syms a b c d t;

% 假设四个矩阵为带有变量的表达式
A = [a*t, b*t^2; c*t^3, d*t];
B = [a*t^2, b*t; c*t, d*t^3];
C = [a*t^3, b*t^2; c*t, d*t];
D = [a*t, b*t^3; c*t^2, d*t];

% 计算矩阵的大小
A_rows = size(A, 1);
A_cols = size(A, 2);
B_rows = size(B, 1);
B_cols = size(B, 2);
C_rows = size(C, 1);
C_cols = size(C, 2);
D_rows = size(D, 1);
D_cols = size(D, 2);

% 将符号表达式转换为函数表达式并计算数值矩阵
A_vals = double(subs(A, [a, b, c, d, t], [1, 2, 3, 4, 0]));
B_vals = double(subs(B, [a, b, c, d, t], [1, 2, 3, 4, 0]));
C_vals = double(subs(C, [a, b, c, d, t], [1, 2, 3, 4, 0]));
D_vals = double(subs(D, [a, b, c, d, t], [1, 2, 3, 4, 0]));

% 计算合并后矩阵的大小
maxRows = max([A_rows, B_rows, C_rows, D_rows]);
maxCols = max([A_cols, B_cols, C_cols, D_cols]);

% 补零操作
A_padded = padarray(A_vals, [maxRows - A_rows, maxCols - A_cols], 0, 'post');
B_padded = padarray(B_vals, [maxRows - B_rows, maxCols - B_cols], 0, 'post');
C_padded = padarray(C_vals, [maxRows - C_rows, maxCols - C_cols], 0, 'post');
D_padded = padarray(D_vals, [maxRows - D_rows, maxCols - D_cols], 0, 'post');

% 合并矩阵
combined_matrix = [A_padded, B_padded; C_padded, D_padded];