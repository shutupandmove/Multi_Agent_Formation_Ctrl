clear all
clc
close all

T = 70; % 仿真时间

%% 智能体
n = 3;                              % 任务空间(维数)
params.N = 5;                       % Agent数量
params.m = 1*ones(params.N,1);     % 质量
params.d = 1*ones(params.N,1);     % 摩擦系数
params.faulted_robot = 4;   % 失效Agent

%% 入度矩阵
% 全入度矩阵（完全图）
params.Bfull= [ -1	0	0	0	1	-1  0   0   1   0; 
                1	-1	0	0   0   0   -1  0   0   1;
                0	1   -1  0   0   1   0   -1  0   0;
                0   0   1   -1  0   0   1   0   -1  0;
                0   0   0   1   -1	0   0   1   0   -1];

% 初始入度矩阵（设定编队结构）
params.B =    [ -1	0	0	0	1	-1	0	0	1   0; 
                1	-1	0	0   0   0   0   0   0   0;
                0	1   -1  0   0   1   0   0   0   0;
                0   0   1   -1  0   0   0   0   -1  0;
                0   0   0   1   -1	0   0   0   0   0];

params.E = size(params.Bfull,2);	% 完全图的边数
E_max = params.N*(params.N-1)/2;    % 图的最大边数
if(params.E < E_max)    % 全入度矩阵错误（边长错误）报错
    fprintf('ERROR: Incedence matrix is not complete. Graph must have %d edges!',E_max);
end
params.dc = 3*ones(params.E,1);	% 阻尼系数
params.kc = 5*ones(params.E,1);	% 弹簧常数5

%% 1号编队设置
%计算完整编队中各边长
d = 20; % 五边形边长（临界距离）
beta = (2*pi)/params.N; % 五边形各边对应中心角
alpha = - beta/2;   % 五边形外角
params.z_des1 = zeros(n*params.E,1); % 各边以点1为原心的三维向量表示
for i = 1:params.E
    if(i<=params.N) % 五条外边情形
        params.z_des1(i*n-n+1:i*n) = ...
            [d*cos(alpha-(i-1)*beta); d*sin(alpha-(i-1)*beta); 0];
    elseif(i>=params.N) % 其他内边情形
        k = i-params.N;
        h = k+1;
        if(h>params.N)
            h = 1;
        end
        params.z_des1(i*n-n+1:i*n) = ...
            params.z_des1(k*n-n+1:k*n) + params.z_des1(h*n-n+1:h*n);
    end
end

%% 2号编队设置
params.z_des2 = zeros(n*params.E,1); % 各边以点1为原心的三维向量表示
d = 20;
params.z_des2(1:n*1)=[0;d;0];
params.z_des2(1+n*1:n*2)=[sqrt(2)*d*cos(-pi/4);sqrt(2)*d*sin(-pi/4);0];
params.z_des2(1+n*2:n*3)=[sqrt(2)*d*cos(-pi*3/4);sqrt(2)*d*sin(-pi*3/4);0];
params.z_des2(1+n*3:n*4)=[sqrt(2)*d*cos(pi*3/4);sqrt(2)*d*sin(pi*3/4);0];
params.z_des2(1+n*4:n*5)=[d;0;0];
params.z_des2(1+n*5:n*6)=[d;0;0];
params.z_des2(1+n*6:n*7)=[0;-2*d;0];
params.z_des2(1+n*7:n*8)=[-2*d;0;0];
params.z_des2(1+n*8:n*9)=[0;d;0];
params.z_des2(1+n*9:n*10)=[sqrt(2)*d*cos(pi/4);sqrt(2)*d*sin(pi/4);0];

%% 能量罐设置
params.tank_size = 100;
params.tank_thresh = 0.25;  % 裕度   
params.tank_max = params.tank_size*(1+params.tank_thresh);  % 上界
params.tank_min = params.tank_size*(1-params.tank_thresh);  % 下界
params.fault_occurred = 0;
params.gamma = 10^(-4)*ones(params.E,1);	% 能量交换的比率

params.gain = zeros(n*params.N,1);
for i = 1:params.N
    params.gain(n*i-n+1:n*i) = ones(n,1)*(1/params.m(i));
end
clear i
%% 参量Params初始化
params_bus_info = Simulink.Bus.createObject(params);
params_bus = evalin('base',params_bus_info.busName);

%% 其他参量初始化
p0 = zeros(n*params.N,1);  

r = 5;
beta = (2*pi)/params.N;
q0 = zeros(n*params.N, 1);  % 以编队几何中心为原点标记各Agent位置
for i = 1:params.N
    q0(n*i-n+1:n*i) = [r*cos(pi/2-(i-1)*beta); r*sin(pi/2-(i-1)*beta); 0];
end

z0 = zeros(n*params.E,1);   % 根据入度矩阵计算各有向边矢量
for i = 1:params.E
    from = find(params.Bfull(:,i)==-1);
    to = find(params.Bfull(:,i)==1);
    z0(i*n-n+1:i*n) = q0(to*n-n+1:to*n)-q0(from*n-n+1:from*n);
end

t0 = params.tank_size*ones(params.N,1);    % 能量罐初始化

initialCondition = [p0; z0; q0; t0];