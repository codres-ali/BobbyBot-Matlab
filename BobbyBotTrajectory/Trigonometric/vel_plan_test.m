clearvars; 

vel_plan = VelocityPlanner('vi',0.6,'vf',0,'a_max',1);

vel_plan.plot_v();

figure(2);

vel_plan.plot_s();