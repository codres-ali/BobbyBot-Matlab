clearvars; 

trj = TrajectoryPlanner('R',1,'dR',0.1,'n',2,'xi',0,'v_max',0.3,'a_max',0.25,'dt',0.15,'phase',0);

trj.PlanTraj_StartRunStop();

figure(1);
trj.plot_xy();
figure(2);
trj.plot_vw();

traj = trj.trajectory;
traj(:,2) = traj(:,2)+1.1;
save('trajectory_spool1.txt','traj','-ascii');
