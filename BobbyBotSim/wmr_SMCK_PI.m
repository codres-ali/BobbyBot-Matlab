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

trj = TrajectoryPlanner('R',1,'dR',0.1,'n',2,'xi',0,'v_max',0.6,'a_max',1,'dt',dt_smc,'phase',0);

trj.PlanTraj_StartRunStop();

smck = SMCKinematic('R',0.05, 'L',0.055, 'k1',6, 'k2',10, 'k0',15, 'P1',0.2, 'P2',0.5, 'Q1',0.5, 'Q2',2, 'init_pose',trj.trajectory(1,1:3)); 

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

count_pid = 0;
count_smc = 0;

k_smc = 1;

for i=1:N
   
    if count_smc == 50
        smck.Control(wR,wL,trj.trajectory(k_smc,:),dt_smc);
        count_smc = 0;
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
    end
    
    if count_pid == 10
        PIRight.Control(smck.wRc,wR,dt_pid);
        PILeft.Control(smck.wLc,wL,dt_pid);
        count_pid = 0;
    end
        
    robot.Integrate(PIRight.u,PILeft.u,dt);
    
    wR = robot.eta(1);
    wL = robot.eta(2);
    
    wR_all(i) = wR;
    wL_all(i) = wL;
    
    count_pid = count_pid + 1;
    count_smc = count_smc + 1;
end

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

