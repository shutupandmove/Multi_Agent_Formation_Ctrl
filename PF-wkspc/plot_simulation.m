%% 绘图基础数据

B = out.B.Data;
N = params.N;   % Agent数量
E = params.E;   % 边数
n = size(out.state.Data,2)/(N+E);	% 任务空间
%% 颜色设置
leader_c = '#EE1111';   % 领队
follower_c = '#2222FF'; % 从机
faulted_c = '#7777FF';  % 失效Agent
trajectory_c = '#006600';   % 轨迹
color = cell(1,N);
color{1} = leader_c;
for i = 2:N
    color{i} = follower_c;
end

%% 获取Agent数据
robot = cell(N,1);
for k = 1:N
    robot{k}.x = (out.q.Data(:,n*(k-1)+1)).';
    robot{k}.y = (out.q.Data(:,n*(k-1)+2)).';
    robot{k}.z = (out.q.Data(:,n*(k-1)+3)).';
end

%% 视频初始化
set(gcf,'units','points','position',[200,200,600,600]);
myVideo = VideoWriter('simulation','MPEG-4'); % 打开新建视频
myVideo.FrameRate = 15;
myVideo.Quality = 100;
open(myVideo);

fault_occurred = 0;
s = size(out.q.Data,1);

%% 检查状态时间常量
for i = 1:s
    exit_flag = 0;
    for k = 1:N
        if( i>s*0.9 && norm([robot{k}.x(i)-robot{k}.x(i-2); robot{k}.y(i)-robot{k}.y(i-2); robot{k}.z(i)-robot{k}.z(i-2)]) < 0.001 )
            exit_flag = exit_flag + 1; % Agent不动了
        end
    end
    if(exit_flag==N)
        s = i;
        break;
    end
end

x_min=robot{1}.x(1); y_min=robot{1}.y(1); z_min=robot{1}.z(1);
x_max=robot{1}.x(1); y_max=robot{1}.y(1); z_max=robot{1}.z(1);
for k = 1:N
    if(max(robot{k}.x) > x_max)
        x_max = max(robot{k}.x)*1.1+1;
    end
    if(min(robot{k}.x) < x_min)
        x_min = min(robot{k}.x)*1.1-1;
    end
    if(max(robot{k}.y) > y_max)
        y_max = max(robot{k}.y)*1.1+1;
    end
    if(min(robot{k}.y) < y_min)
        y_min = min(robot{k}.y)*1.1-1;
    end
	if(max(robot{k}.z) > z_max)
        z_max = max(robot{k}.z)*1.1+1;
    end
    if(min(robot{k}.z) < z_min)
        z_min = min(robot{k}.z)*1.1;
    end
end

for i = 1:s
    %% 绘制边
    for edge = 1:E
        if(size(find(B(:,edge,i)==1),1) ~= 0)
            f = find(B(:,edge,i)==-1);
            t = find(B(:,edge,i)==1);
            line([robot{f}.x(i),robot{t}.x(i)], [robot{f}.y(i),robot{t}.y(i)], [robot{f}.z(i),robot{t}.z(i)], 'Color','#111111');
        end
        hold on;
    end
    
    %% 绘制轨迹
    plot3(out.ref.Data(:,1),out.ref.Data(:,2),out.ref.Data(:,3), 'Color',trajectory_c,'linewidth',1.2);
    plot3(out.ref.Data(i,1),out.ref.Data(i,2),out.ref.Data(i,3), '.','Color',trajectory_c,'MarkerSize',22);
    
    %% 绘制Agent
    plot3(robot{1}.x(1:i), robot{1}.y(1:i), robot{1}.z(1:i), 'Color', color{1},'linewidth',1.2);
    for k = 1:N
        if(sum(abs(B(k,:,i)))==0 && ~fault_occurred)
            fault_occurred = 1;
            color{k} = faulted_c;
        end
        plot3(robot{k}.x(i), robot{k}.y(i), robot{k}.z(i),'.','Color',color{k},'MarkerSize',30);
        hold on;
        
        % check if robot at steady state
        if ( i>100 && norm([robot{k}.x(i)-robot{k}.x(i-2); robot{k}.y(i)-robot{k}.y(i-2); robot{k}.z(i)-robot{k}.z(i-2)]) < 0.1 )
            exit_flag = exit_flag + 1;  % robot is not moving anymore
        end
    end
    
    set(gca,'XLim',[x_min, x_max],'YLim',[y_min, y_max],'ZLim',[z_min, z_max+1]);
    grid on, view(10,60);
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
    clf;
    
end

close(myVideo)
close all
clear myVideo frame
