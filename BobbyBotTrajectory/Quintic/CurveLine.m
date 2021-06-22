classdef CurveLine < handle
    properties
        P0;
        P1;
        m;
        s_max = 0;
        dt = 0.01;
    end
    
    methods
        function obj=CurveLine(varargin)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'P0'
                        obj.P0 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'P1'
                        obj.P1 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dt'
                        obj.dt = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            
            obj.m = (obj.P1(2)-obj.P0(2))/(obj.P1(1)-obj.P0(1));
            
            obj.s_max = ((obj.P1(1)-obj.P0(1))^2+(obj.P1(2)-obj.P0(2))^2)^0.5;
        end
        
        function [x,y]=xy(obj,t)  
            x = 0;
            y = 0;
            if t>=0 && t<=1
                x = obj.P0(1)+(obj.P1(1)-obj.P0(1))*t;
                y = obj.P0(2)+(obj.P1(2)-obj.P0(2))*t;
            end
        end
        
        function [x_dot,y_dot]=xy_dot(obj,t)  
            x_dot = 0;
            y_dot = 0;
            if t>=0 && t<=1
                x_dot = obj.P1(1)-obj.P0(1);
                y_dot = obj.P1(2)-obj.P0(2);
            end
        end
        
        function [x_dot_dot,y_dot_dot]=xy_dot_dot(obj,t)  
            x_dot_dot = 0;
            y_dot_dot = 0;
        end
                
        function theta=theta(obj,t)  
            theta = 0;
            if t>=0 && t<=1
                [x_dot, y_dot] = obj.xy_dot(t);
                theta = wrapTo2Pi(atan2(y_dot,x_dot));
            end
        end
        
        function k=k(obj,t)  
            k = 0;
        end
        
        function k_dot=k_dot(obj,t)  
            k_dot = 0;
        end
                
        function s=s_arc(obj,t)  
            s = 0;
            if t>=0 && t<=1
                [x,y]=xy(obj,t);
                s = ((x-obj.P0(1))^2+(y-obj.P0(2))^2)^0.5;
            end
        end
        
        function t=t_arc(obj,s)  
            t = 0;
            if s>=0 && s<=obj.s_max
                t = s/obj.s_max;
            else
                if s>obj.s_max
                    t = 1;
                end
            end
            if t>1
                t = 1;
            end
        end
        
        function plot_xy(obj)
            t = 0:0.01:1;
            xy_all = zeros(2,size(t,1));
            for i=1:size(t,2)
                [xy_all(1,i),xy_all(2,i)] = obj.xy(t(i));
            end
            plot(xy_all(1,:),xy_all(2,:),'LineWidth',2);
        end
        
        function plot_k(obj)
            t = 0:0.01:1;
            k_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                k_all(i) = obj.k(t(i));
            end
            plot(t,k_all,'LineWidth',2);
        end
        
        function plot_k_dot(obj)
            t = 0:0.01:1;
            k_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                k_all(i) = obj.k_dot(t(i));
            end
            plot(t,k_all,'LineWidth',2);
        end
        
        function plot_theta(obj)
            t = 0:0.01:1;
            theta_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                theta_all(i) = obj.theta(t(i));
            end
            plot(t,theta_all,'LineWidth',2);
        end
        
     end
end
