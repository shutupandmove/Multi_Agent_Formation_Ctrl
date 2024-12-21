运行顺序：
Init_5N.m(参数初始化)-->formation_control.slx(运行仿真)-->plot_simulation.m(取数据绘制并本地生成视频simulation.mp4)

关键参数：

Init_5N.m
	line37-71，编队队形设计（任何新设计的队形都最好符合line14-28的入度矩阵，不然就自己重写）

formation_control.slx
	Manual Switch(fault_signal)，是否让4号机中途失效
	Manual Switch(pose_signal_gennerator)，选择什么样的轨迹
	Multi-Robot System(line91)，是否启用轨迹跟踪开关
	trajectory，设计的参考轨迹

