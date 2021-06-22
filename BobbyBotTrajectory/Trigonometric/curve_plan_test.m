clearvars; 

crv_plan1 = CurvePlanner('R',1,'dR',0.2,'n',4,'xi',0);
crv_plan2 = CurvePlanner('R',1,'dR',0.2,'n',4,'xi',pi);

figure(1);
crv_plan1.plot_xy();
hold on;
axis equal;
crv_plan2.plot_xy();

figure(2);
crv_plan1.plot_k_dot();

