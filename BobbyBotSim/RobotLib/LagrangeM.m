classdef LagrangeM < handle
 % Class for simulating a WMR model using Lagrange method    
    properties
         
        %input parameters
        TiR = 0;
        TiL = 0;
        VaR = 0;
        VaL = 0;
        
        % set which input to use
        % 1 - Voltage input
        % 2 - Torque input
        input = 1;
        
        % with/without friction
        friction = 1
        
        % DC motor parameters
        KT = 0.01;
        Ke = 0.01;
        Rar = 3;
        Ral = 3;
        La = 0.01;
        
        % reduction gear parameters
        tr = 34;        
        miu = 1;%73/100;
        tau_fr = 0;%5.6*10^-3;
        
        % robot parameters 
        Rr = 0.05;          % wheel radius
        Rl = 0.05;
        L = 0.055;          % half of track width
        d = 0;              % distance between center of mass and center of rotation
        M = 0.6;            % mass of the robot
        mw = 0.05;          % mass of the wheel

        % moment of inertia
        I0 = 0.001;
        Im = 0.0002;
        Iw = 0.0002;
        
        % friction parameters (static and viscous for each wheel)
        Cs_r = 0.017;
        Cs_l = 0.017;
        Cv_r = 0.0002;
        Cv_l = 0.0002;
        
        % output velocity for right and left wheel
        eta = [0;0];
        
        xr,yr,thetar,vr,wr,Ir,Il,Ir_dot,Il_dot,tauR,tauL;
    end
    
    methods
        function obj=LagrangeM()
            % initialize variables
            obj.xr=0;
            obj.yr=0;
            obj.thetar=0;
            obj.vr=0;
            obj.wr=0;
            obj.Ir = 0;
            obj.Il = 0;
            obj.Ir_dot = 0;
            obj.Il_dot = 0;
        end
        
        function obj=Integrate(obj,uR,uL,dt)
            
            obj.VaR = 5*uR;
            obj.VaL = 5*uL;
            
            if obj.friction == 1
                d1 = obj.d;
            else
                d1 = 0.0;
            end
            
            % Create M matrix for the Lagrange model (Inertia matrix)
            Mt = obj.M+2*obj.mw;
            It = obj.I0+obj.M*d1^2+2*obj.mw*obj.L^2+2*obj.Im;

            m11 = obj.Rr^2/4*(Mt+It/obj.L^2)+obj.Iw;
            m12 = obj.Rr*obj.Rl/4*(Mt-It/obj.L^2);
            m21 = m12;
            m22 = obj.Rl^2/4*(Mt+It/obj.L^2)+obj.Iw;

            Mq = [m11 m12; m21 m22];
            
            % Create V matrix for the Lagrange model (coriolis matrix)
            v11 = 0;
            v12 = obj.M*d1*obj.Rr*obj.Rl*obj.wr/(2*obj.L);
            v21 = -obj.M*d1*obj.Rr*obj.Rl*obj.wr/(2*obj.L);
            v22 = 0;

            Vq = [v11 v12; v21 v22];

            % Torque input matrix
            Bq = [1 0; 0 1];

            % Friction for each wheel
            fr1 = obj.Cs_r*sign(obj.eta(1))+obj.Cv_r*obj.eta(1);
            fr2 = obj.Cs_l*sign(obj.eta(2))+obj.Cv_l*obj.eta(2);

            if obj.friction == 1
                Fr = [fr1;fr2];
            else
                Fr = [0;0];
            end

            % simulate the DC motor for each wheel
            obj.Ir_dot = (obj.VaR - obj.Ke*obj.tr*obj.eta(1) - obj.Rar*obj.Ir)/obj.La;
            obj.Ir = obj.Ir + dt*obj.Ir_dot;
            obj.Il_dot = (obj.VaL - obj.Ke*obj.tr*obj.eta(2) - obj.Ral*obj.Il)/obj.La;
            obj.Il = obj.Il + dt*obj.Il_dot;

            obj.tauR = obj.tr*obj.miu*(obj.KT*obj.Ir - obj.tau_fr);
            obj.tauL = obj.tr*obj.miu*(obj.KT*obj.Il - obj.tau_fr);
    
            % Select the input 
            %   1 - torque from DC motor;
            %   2 - torque from model input;
            if obj.input == 1
                Tau = [obj.tauR;obj.tauL];
            end;
            if obj.input == 2
                Tau = [obj.TiR;obj.TiL];
            end;

            % Lagrange formula for the model - computes acceleration for
            % each wheel
            eta_dot = inv(Mq)*(Bq*Tau - Vq*obj.eta - Fr);

            % integrate to get the velocity for each wheel
            obj.eta = obj.eta + eta_dot*dt;

            % compute robot velocities and pose
            obj.vr = (obj.Rr*obj.eta(1)+obj.Rl*obj.eta(2))/2;
            obj.wr = (obj.Rr*obj.eta(1)-obj.Rl*obj.eta(2))/(2*obj.L);

            obj.thetar = obj.thetar + obj.wr*dt;

            obj.xr = obj.xr + dt*obj.vr*cos(obj.thetar);
            obj.yr = obj.yr + dt*obj.vr*sin(obj.thetar);
        end
    end
    
end


