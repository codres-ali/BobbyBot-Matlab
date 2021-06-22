classdef Robot < handle
    properties
	     robot_model
		 ip = '192.168.1.1';
		 robot_com
		 run_mode = 'RealRobot';
		 dt = 0.01;
         tloop = tic;
    end
    
    methods
        function obj=Robot(varargin)
            % Constructor
            % order of inputs shouldn't matter (check later)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'ip'
                        % Read json-file
                        obj.ip = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'mode'
                        obj.run_mode = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
		    if(strcmp(obj.run_mode,'RobotSimulation'))
              obj.robot_model = LagrangeM;
			end
		    if(strcmp(obj.run_mode,'RealRobot'))
              obj.robot_com = Connect(obj.ip,3141,'udp');
            end
            pause(1);
            obj.tloop = tic;
        end
		
        function [output]=Run(obj,input)				    
			if(strcmp(obj.run_mode,'RealRobot'))
                key = keys(input);
                val = values(input);
                for i=1:size(input,1)
                    msg.data = val{i};
                    msg.stamp = GetTime(obj.robot_com);
                    SendTopicMsg(obj.robot_com,key{i},msg);
                end
				obj.robot_com = SpinOnce(obj.robot_com);
                
                output = obj.robot_com.msgs;

			end
			
			if(strcmp(obj.run_mode,'RobotSimulation'))
                uR = 0;
                uL = 0;
                
                if input.isKey('ControlR')
                    uR = input('ControlR');
                end
                if input.isKey('ControlL')
                    uL = input('ControlL');
                end
                
                obj.robot_model.Integrate(uR,uL,obj.dt);
                
                output = containers.Map();
                msg.data = obj.robot_model.eta(1);
                msg.status = 1;
                output('VelocityEncR') = msg;
                msg.data = obj.robot_model.eta(2);
                msg.status = 1;
                output('VelocityEncL') = msg;
            end
            
            t_pause = obj.dt-toc(obj.tloop);
            if(t_pause>0)
                pause(t_pause);
            end
            obj.tloop = tic;
        end
		
        function Stop(obj)
		    if(strcmp(obj.run_mode,'RealRobot'))
              Disconnect(obj.robot_com);
			end
        end
        function SetSamplingTime(obj,dt)
		   obj.dt = dt;
        end
    end
    
end

