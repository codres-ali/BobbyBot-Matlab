restoredefaultpath
addpath(genpath(pwd))
clearvars;

wR = 0;
wL = 0;

wR_set = 0;
wL_set = 0;

dt = 0.001;
dt_pid = 0.01;
dt_smc = 0.05;

robot = LagrangeM;

PIRight = PIDControl;
PILeft = PIDControl;

trj = TrajectoryPlanner('R',0.8,'dR',0.2,'n',4,'xi',0,'v_max',0.4,'a_max',0.1,'dt',dt_smc,'phase',0,'dir',-1);

trj.PlanTraj_StartRunStop();

smck = SMCKinematic('R',0.05, 'L',0.055, 'k1',10, 'k2',13, 'k0',20, 'P1',0.3, 'P2',0.7, 'Q1',0.5, 'Q2',2, 'init_pose',trj.trajectory(1,1:3)); 

N = size(trj.trajectory,1)*50;

wR_all = zeros(1,N);
wL_all = zeros(1,N);
xr_all = zeros(1,size(trj.trajectory,1)-1);
yr_all = zeros(1,size(trj.trajectory,1)-1);
thetar_all = zeros(1,size(trj.trajectory,1)-1);
xe_all = zeros(1,size(trj.trajectory,1)-1);
ye_all = zeros(1,size(trj.trajectory,1)-1);
thetae_all = zeros(1,size(trj.trajectory,1)-1);
vr_all = zeros(1,size(trj.trajectory,1)-1);
wr_all = zeros(1,size(trj.trajectory,1)-1);
wRc_all = zeros(1,size(trj.trajectory,1)-1);
wLc_all = zeros(1,size(trj.trajectory,1)-1);

t = 0:dt:1-dt;

k_smc = 1;
i=1;

figure(1);
        
t_start = tic;
t_loop_smc = tic;
t_loop_pid = tic;

while k_smc<=size(trj.trajectory,1)
   
    dt1 = toc(t_loop_smc);
    
    if(dt1>=dt_smc) 
        t_loop_smc = tic;
        
        smck.Control(wR,wL,trj.trajectory(k_smc,:),dt_smc);

        xr_all(k_smc) = smck.xr;
        yr_all(k_smc) = smck.yr;
        thetar_all(k_smc) = smck.thetar;
        xe_all(k_smc) = smck.x_e;
        ye_all(k_smc) = smck.y_e;
        thetae_all(k_smc) = smck.theta_e;
        vr_all(k_smc) = smck.vr;
        wr_all(k_smc) = smck.wr;
        wRc_all(k_smc) = smck.wRc;
        wLc_all(k_smc) = smck.wLc;
        k_smc = k_smc +1;

        hold off;
        plot(trj.trajectory(:,1),trj.trajectory(:,2),':r','LineWidth',2);
        axis([-1.5 1.5 -1.5 1.5]);
        hold on;
        rectangle('Position',[smck.xr-0.075 smck.yr-0.075 0.15 0.15],'Curvature',[1 1]);
    end
    
    dt1 = toc(t_loop_pid);
    
    if(dt1>=dt_pid) % execute code when desired sampling time is reached
        t_loop_pid = tic;
        PIRight.Control(smck.wRc,wR,dt_pid);
        PILeft.Control(smck.wLc,wL,dt_pid);
    end
        
    robot.Integrate(PIRight.u,PILeft.u,dt);
    
    wR = robot.eta(1);
    wL = robot.eta(2);
    
    wR_all(i) = wR;
    wL_all(i) = wL;
    
    i = i+1;
        
    pause(1e-3);
end
toc(t_start);

figure(1);
plot(trj.trajectory(:,1),trj.trajectory(:,2),':r','LineWidth',2);
hold on;
plot(xr_all,yr_all,'LineWidth',2);
figure(2);
plot(vr_all,'LineWidth',2);
hold on;
plot(wr_all,'LineWidth',2);
figure(3);
plot(wRc_all,'LineWidth',2);
hold on;
plot(wLc_all,'LineWidth',2);
figure(4);
plot(xe_all,'LineWidth',2);
hold on;
plot(ye_all,'LineWidth',2);
plot(thetae_all,'LineWidth',2);

