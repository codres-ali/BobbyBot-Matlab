classdef TrajectoryPlanner < handle
    properties
	     trajectory
         curve
         vel_start
         vel_stop
         vel_stop_fast
         dt = 0.05;
         v_max = 0.5;
         a_max = 1;
         dir = 1;
         phase = 0;
    end
    
    methods
        function obj=TrajectoryPlanner(varargin)
            var_ind = 1;
            R = 1;
            dR = 0.1;
            n = 2;
            xi = 0;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'R'
                        R = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dR'
                        dR = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'n'
                        n = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'xi'
                        xi = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dt'
                        dt = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'v_max'
                        obj.v_max = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'a_max'
                        obj.a_max = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dir'
                        obj.dir = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'phase'
                        obj.phase = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            
            obj.curve = CurvePlanner('R',R,'dR',dR,'n',n,'xi',xi,'dir',obj.dir);
            obj.vel_start = VelocityPlanner('vi',0,'vf',obj.v_max,'a_max',obj.a_max);
            obj.vel_stop = VelocityPlanner('vi',obj.v_max,'vf',0,'a_max',-obj.a_max);
            obj.vel_stop_fast = VelocityPlanner('vi',obj.v_max,'vf',0,'a_max',-2*obj.a_max);
        end
        
        function PlanTraj_StartRunStop(obj)
            obj.trajectory = [];
            k = 0;
            tc = 0;
            first = true;
            exit = false;
            s_start = obj.curve.s_arc(obj.phase);
            while tc<2*pi && ~exit
                t = k*obj.dt;
                if t < obj.vel_start.tf
                    obj.trajectory(k+1,4) = obj.vel_start.v(t);
                    obj.trajectory(k+1,6) = obj.vel_start.a(t);
                    s = s_start+obj.vel_start.s(t);
                else
                    if s<(obj.curve.s_max-obj.vel_stop.s_max-2*obj.dt*obj.v_max)
                        obj.trajectory(k+1,4) = obj.v_max;
                        obj.trajectory(k+1,6) = 0;
                        s = s + obj.dt*obj.v_max;
                    else
                        if first
                            t_stop = t;
                            s_stop = s+obj.dt*obj.v_max;
                            first = false;
                        end
                        ts = t - t_stop;
                        if ts <= obj.vel_stop.tf 
                            obj.trajectory(k+1,4) = obj.vel_stop.v(ts);
                            obj.trajectory(k+1,6) = obj.vel_stop.a(ts);
                            s = s_stop+obj.vel_stop.s(ts);
                        else
                            exit = true;
                        end
                    end
                end
                
                if ~exit
                    tc = obj.curve.t_arc(s);
                    [obj.trajectory(k+1,1),obj.trajectory(k+1,2)] = obj.curve.xy(tc);
                    obj.trajectory(k+1,3) = obj.curve.theta(tc);
                    obj.trajectory(k+1,5) = obj.trajectory(k+1,4)*obj.curve.k(tc);
                    obj.trajectory(k+1,7) = obj.trajectory(k+1,6)*obj.curve.k_dot(tc);
                    obj.trajectory(k+1,8) = t;
                end
                
                k = k+1;
            end            
        end
        
        function PlanTraj_StartRun(obj)
            obj.trajectory = [];
            k = 0;
            tc = 0;
            s_start = obj.curve.s_arc(obj.phase);
            while tc<2*pi
                t = k*obj.dt;
                if t < obj.vel_start.tf
                    obj.trajectory(k+1,4) = obj.vel_start.v(t);
                    obj.trajectory(k+1,6) = obj.vel_start.a(t);
                    s = s_start+obj.vel_start.s(t);
                else
                    obj.trajectory(k+1,4) = obj.v_max;
                    obj.trajectory(k+1,6) = 0;
                    s = s + obj.dt*obj.v_max;
                end
                

                    tc = obj.curve.t_arc(s);
                    [obj.trajectory(k+1,1),obj.trajectory(k+1,2)] = obj.curve.xy(tc);
                    obj.trajectory(k+1,3) = obj.curve.theta(tc);
                    obj.trajectory(k+1,5) = obj.trajectory(k+1,4)*obj.curve.k(tc);
                    obj.trajectory(k+1,7) = obj.trajectory(k+1,6)*obj.curve.k_dot(tc);
                    obj.trajectory(k+1,8) = t;
                
                k = k+1;
            end            
        end
        
        function PlanTraj_Run(obj)
            obj.trajectory = [];
            k = 0;
            tc = 0;
            s_start = obj.curve.s_arc(obj.phase);
            s = s_start;
            while tc<2*pi
                t = k*obj.dt;
                    
                obj.trajectory(k+1,4) = obj.v_max;
                obj.trajectory(k+1,6) = 0;
                s = s + obj.dt*obj.v_max;
                
                    tc = obj.curve.t_arc(s);
                    [obj.trajectory(k+1,1),obj.trajectory(k+1,2)] = obj.curve.xy(tc);
                    obj.trajectory(k+1,3) = obj.curve.theta(tc);
                    obj.trajectory(k+1,5) = obj.trajectory(k+1,4)*obj.curve.k(tc);
                    obj.trajectory(k+1,7) = obj.trajectory(k+1,6)*obj.curve.k_dot(tc);
                    obj.trajectory(k+1,8) = t;
                
                k = k+1;
            end            
        end
        
        function plot_xy(obj)
            plot(obj.trajectory(:,1),obj.trajectory(:,2));
        end
        function plot_vw(obj)
            plot(obj.trajectory(:,4));
            hold on;
            plot(obj.trajectory(:,5));
        end
    end
end
