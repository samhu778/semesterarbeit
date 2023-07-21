clear all
clc;
% 创建串口对象
port = 'COM3';  % 根据实际情况选择串口号
baudrate = 115200;  % 设置波特率
imuSerial = serialport(port, baudrate);

% 初始化变量
timeStamp = [];

% 接收数据并记录时间
disp('开始接收数据...');
while true
    % 读取串口数据
    data = readline(imuSerial);
    
    % 记录时间戳
    currentTime = datetime('now');
    timeStamp = [timeStamp, currentTime];
    
    % 处理接收到的数据（这里可以根据实际情况进行相应的处理）
    % ...
    
    % 终止条件（这里可以根据实际情况进行设定）
    if numel(timeStamp) >= 100
        break;
    end
end

% 关闭串口
close(imuSerial);

% 显示时间戳
disp('接收完成，时间戳记录如下：');
disp(timeStamp);