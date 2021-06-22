restoredefaultpath
addpath(genpath(pwd))
clearvars;

dt = 0.001;
dt_pid = 0.01;
dt_smc = 0.05;

n_robots = 8;

for i=1:n_robots/2
    phase = (i-1)*4*pi/n_robots;
    trj(i) = TrajectoryPlanner('R',1.2,'dR',0.2,'n',floor(n_robots/4),'xi',0,'v_max',0.5,'a_max',0.2,'dt',dt_smc,'phase',phase,'dir',-1);

    trj(i).PlanTraj_StartRun();
end
for i=n_robots/2+1:n_robots
    phase = wrapTo2Pi(2*pi/n_robots+(i-n_robots/2-1)*4*pi/n_robots);
    trj(i) = TrajectoryPlanner('R',1.2,'dR',0.2,'n',floor(n_robots/4),'xi',pi,'v_max',0.5,'a_max',0.2,'dt',dt_smc,'phase',phase);

    trj(i).PlanTraj_StartRun();
end
for i=1:n_robots
    robot(i) = LagrangeM;

    PIRight(i) = PIDControl;
    PILeft(i) = PIDControl;

    smck(i) = SMCKinematic('R',0.05, 'L',0.055, 'k1',10, 'k2',14, 'k0',20, 'P1',0.3, 'P2',0.7, 'Q1',0.5, 'Q2',2, 'init_pose',trj(i).trajectory(1,1:3)); 
end

wR = zeros(n_robots);
wL = zeros(n_robots);

N = size(trj(1).trajectory,1)*50;

k_smc = ones(n_robots);
i=1;

figure(1);
        
t_start = tic;
t_loop_smc = tic;
t_loop_pid = tic;

while k_smc(1)<=size(trj(1).trajectory,1)
   
    dt1 = toc(t_loop_smc);
    
    if(dt1>=dt_smc) 
        t_loop_smc = tic;
        
        hold off;
        plot(trj(1).trajectory(:,1),trj(1).trajectory(:,2),':r','LineWidth',2);
        axis([-2 2 -2 2]);
        hold on;
        plot(trj(n_robots/2+1).trajectory(:,1),trj(n_robots/2+1).trajectory(:,2),':b','LineWidth',2);
       
        for k=1:n_robots
            smck(k).Control(wR(k),wL(k),trj(k).trajectory(k_smc(k),:),dt_smc);
            k_smc(k) = k_smc(k) +1;
            if k_smc(k) > size(trj(k).trajectory,1)
                k_smc(k) = 1;

                trj(k).phase = 0;
                trj(k).PlanTraj_Run();
            end

            rectangle('Position',[smck(k).xr-0.1 smck(k).yr-0.1 0.2 0.2],'Curvature',[1 1]);
        end
    end
    
    dt1 = toc(t_loop_pid);
    
    if(dt1>=dt_pid) % execute code when desired sampling time is reached
        t_loop_pid = tic;
        for k=1:n_robots
            PIRight(k).Control(smck(k).wRc,wR(k),dt_pid);
            PILeft(k).Control(smck(k).wLc,wL(k),dt_pid);
        end
    end
        
    for k=1:n_robots
        robot(k).Integrate(PIRight(k).u,PILeft(k).u,dt);

        wR(k) = robot(k).eta(1);
        wL(k) = robot(k).eta(2);
    end
            
    pause(1e-3);
end
toc(t_start);

