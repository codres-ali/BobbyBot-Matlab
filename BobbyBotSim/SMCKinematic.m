classdef SMCKinematic < handle
    properties
         
        xr = 0;
        yr = 0;
        thetar = 0;
        vr = 0;
        wr = 0;
        vc = 0;
        wc = 0;
        wRc = 0;
        wLc = 0;
        R,L,k0,k1,k2,P1,P2,Q1,Q2,x_e,y_e,theta_e
    end
    
    methods
        function obj=SMCKinematic(varargin)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'R'
                        obj.R = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'L'
                        obj.L = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'k0'
                        obj.k0 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'k1'
                        obj.k1 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'k2'
                        obj.k2 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'P1'
                        obj.P1 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'P2'
                        obj.P2 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'Q1'
                        obj.Q1 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'Q2'
                        obj.Q2 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'init_pose'
                        pose = varargin{var_ind + 1};
                        obj.xr = pose(1);
                        obj.yr = pose(2);
                        obj.thetar = pose(3);
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
        end
        
        function obj=Control(obj,wR,wL,traj,dt)
            x_d = traj(1);     
            y_d = traj(2);    
            theta_d = traj(3);
            v_d = traj(4);      
            w_d = traj(5);
            v_d_dot = traj(6); 
            w_d_dot = traj(7);

            % (6) Compute the robot velocities (linear and angular)
            obj.vr = (obj.R*wR+obj.R*wL)/2;
            obj.wr = (obj.R*wR-obj.R*wL)/(2*obj.L);

            obj.xr = obj.xr + dt*obj.vr*cos(obj.thetar+obj.wr*dt/2);
            obj.yr = obj.yr + dt*obj.vr*sin(obj.thetar+obj.wr*dt/2);
            obj.thetar = wrapTo2Pi(obj.thetar + obj.wr*dt);
            
            % (7) Generate the error signal of each state
            obj.x_e = (obj.xr-x_d)*cos(theta_d)+(obj.yr-y_d)*sin(theta_d);
            obj.y_e = -(obj.xr-x_d)*sin(theta_d)+(obj.yr-y_d)*cos(theta_d);
            obj.theta_e = wrapToPi(obj.thetar-theta_d);

            % (8) Define the derivatives
            x_e_dot = obj.vr*cos(obj.theta_e) + obj.y_e*w_d - v_d;
            y_e_dot = obj.vr*sin(obj.theta_e) - obj.x_e*w_d;
            theta_e_dot = obj.wr - w_d;

            % (10) Compute sliding surface
            s1 = x_e_dot + obj.k1*obj.x_e;
            s2 = theta_e_dot + obj.k2*obj.theta_e + obj.k0*obj.y_e;

            % (11) Compute control signals
            T1 = obj.vr*theta_e_dot*sin(obj.theta_e)-w_d_dot*obj.y_e-w_d*y_e_dot+v_d_dot-obj.k1*x_e_dot;
            T2 = w_d_dot - obj.k2*theta_e_dot - obj.k0*y_e_dot;

            vc_dot = (-obj.Q1*s1-obj.P1*sign(s1)+T1)/cos(obj.theta_e);
            wc_dot = -obj.Q2*s2-obj.P2*sign(s2)+T2;
 
            obj.vc = obj.vc+dt*vc_dot;
            obj.wc = obj.wc+dt*wc_dot;

            obj.wRc = (obj.vc + obj.L*obj.wc)/obj.R;
            obj.wLc = (obj.vc - obj.L*obj.wc)/obj.R;
            
        end
    end
    
end

