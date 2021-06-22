classdef CurvePlanner < handle
    properties
         R = 0;
         dR = 0;
         n = 0;
         xi = 0;
         s = [];
         s_max = 0;
         dt = 0.01;
    end
    
    methods
        function obj=CurvePlanner(varargin)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'R'
                        obj.R = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dR'
                        obj.dR = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'n'
                        obj.n = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'xi'
                        obj.xi = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dt'
                        obj.dt = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            
            if obj.R > 0 && obj.dR>0 && obj.n>0
                t = 0:obj.dt:2*pi+obj.dt;
                obj.s = zeros(1,size(t,2));
                [x_dot,y_dot] = obj.xy_dot(t(1));
                dsk = (x_dot^2 + y_dot^2)^0.5;
                for k=2:size(t,2)
                    dsk_1 = dsk;
                    [x_dot,y_dot] = obj.xy_dot(t(k));
                    dsk = (x_dot^2 + y_dot^2)^0.5;
                    obj.s(k) = obj.s(k-1)+obj.dt*(dsk+dsk_1)/2;
                end
                obj.s_max = obj.s_arc(2*pi);
            end
        end
        
        function [x,y]=xy(obj,t)  
            x = 0;
            y = 0;
            if t>=0 && t<=2*pi
                x = (obj.R+obj.dR*cos(obj.xi+obj.n*t))*sin(t);
                y = -(obj.R+obj.dR*cos(obj.xi+obj.n*t))*cos(t);
            end
        end
        
        function [x_dot,y_dot]=xy_dot(obj,t)  
            x_dot = 0;
            y_dot = 0;
            if t>=0 && t<=2*pi
                x_dot = cos(t)*(obj.R + obj.dR*cos(obj.xi+obj.n*t)) - obj.dR*obj.n*sin(obj.xi+obj.n*t)*sin(t);
                y_dot = sin(t)*(obj.R + obj.dR*cos(obj.xi+obj.n*t)) + obj.dR*obj.n*sin(obj.xi+obj.n*t)*cos(t);
            end
        end
        
        function [x_dot_dot,y_dot_dot]=xy_dot_dot(obj,t)  
            x_dot_dot = 0;
            y_dot_dot = 0;
            if t>=0 && t<=2*pi
                x_dot_dot = - sin(t)*(obj.R + obj.dR*cos(obj.xi+obj.n*t)) - obj.dR*obj.n^2*cos(obj.xi+obj.n*t)*sin(t) - 2*obj.dR*obj.n*sin(obj.xi+obj.n*t)*cos(t);
                y_dot_dot = cos(t)*(obj.R + obj.dR*cos(obj.xi+obj.n*t)) + obj.dR*obj.n^2*cos(obj.xi+obj.n*t)*cos(t) - 2*obj.dR*obj.n*sin(obj.xi+obj.n*t)*sin(t);
%                 y_dot_dot = 2*obj.dR*obj.n*sin(obj.xi+obj.n*t)*sin(t) - obj.dR*obj.n^2*cos(obj.xi+obj.n*t)*cos(t) - cos(t)*(obj.R + obj.dR*cos(obj.xi+obj.n*t));
            end
        end
        
        function [x_dot_dot_dot,y_dot_dot_dot]=xy_dot_dot_dot(obj,t)  
            x_dot_dot_dot = 0;
            y_dot_dot_dot = 0;
            if t>=0 && t<2*pi
                x_dot_dot_dot = obj.dR*obj.n^3*sin(obj.xi + obj.n*t)*sin(t) - cos(t)*(obj.R + obj.dR*cos(obj.xi + obj.n*t)) + 3*obj.dR*obj.n*sin(obj.xi + obj.n*t)*sin(t) - 3*obj.dR*obj.n^2*cos(obj.xi + obj.n*t)*cos(t);
                y_dot_dot_dot = -sin(t)*(obj.R + obj.dR*cos(obj.xi + obj.n*t)) - 3*obj.dR*obj.n*sin(obj.xi + obj.n*t)*cos(t) - 3*obj.dR*obj.n^2*cos(obj.xi + obj.n*t)*sin(t) - obj.dR*obj.n^3*sin(obj.xi + obj.n*t)*cos(t);
            end
        end
        
        function theta=theta(obj,t)  
            theta = 0;
            if t>=0 && t<=2*pi
                [x_dot, y_dot] = obj.xy_dot(t);
                theta = wrapTo2Pi(atan2(y_dot,x_dot));
            end
        end
        
        function k=k(obj,t)  
            k = 0;
            if t>=0 && t<=2*pi
                [x_dot, y_dot] = obj.xy_dot(t);
                [x_dot_dot, y_dot_dot] = obj.xy_dot_dot(t);
                k = (x_dot*y_dot_dot-x_dot_dot*y_dot)/(x_dot^2+y_dot^2)^1.5;
            end
        end
        
        function k_dot=k_dot(obj,t)  
            k_dot = 0;
            if t>=0 && t<=2*pi
%                 [x_dot, y_dot] = obj.xy_dot(t);
%                 [x_dot_dot, y_dot_dot] = obj.xy_dot_dot(t);
%                 [x_dot_dot_dot, y_dot_dot_dot] = obj.xy_dot_dot_dot(t);
%                 k_dot = ((x_dot*y_dot_dot_dot-x_dot_dot_dot*y_dot)*(x_dot^2+y_dot^2)^1.5+(x_dot*y_dot_dot-x_dot_dot*y_dot)*1.5*(x_dot^2+y_dot^2)^0.5*(2*x_dot*x_dot_dot+2*y_dot*y_dot_dot))/(x_dot^2+y_dot^2)^3;
%                 (x_dot*y_dot_dot-x_dot_dot*y_dot)' = x_dot*y_dot_dot_dot-x_dot_dot_dot*y_dot;
%                 (x_dot^2+y_dot^2)^1.5'= 1.5*(x_dot^2+y_dot^2)^0.5*(2*x_dot*x_dot_dot+2*y_dot*y_dot_dot)
                k_dot = (obj.k(t+0.001)-obj.k(t))/0.001;
            end
        end
                
        function s=s_arc(obj,t)  
            s = 0;
            if t>=0 && t<=2*pi
                k = floor(t/obj.dt);
                s = obj.s(k+1) + (obj.s(k+2)-obj.s(k+1))*(t-k*obj.dt)/obj.dt;
            end
        end
        
        function t=t_arc(obj,s)  
            t = 0;
            if s>=0 && s<=obj.s(end)
                indx = floor(size(obj.s,2)/2);
                up = size(obj.s,2);
                low = 1;
                exit = 0;
                while exit==0
                    if s>=obj.s(indx) && s<obj.s(indx+1)
                        t = (indx-1)*obj.dt + (s-obj.s(indx))*obj.dt/(obj.s(indx+1)-obj.s(indx));
                        exit = 1;
                    else
                        if s<obj.s(indx)
                            up = indx;
                            indx = floor((low+indx)/2);
                        else
                            low = indx;
                            indx = floor((up+indx+1)/2);
                        end
                    end
                end
            else
                if s>obj.s(end)
                    t = 2*pi;
                end
            end
            if t>2*pi
                t = 2*pi;
            end
        end
        
        function plot_xy(obj)
            t = 0:0.01:2*pi;
            xy_all = zeros(2,size(t,1));
            for i=1:size(t,2)
                [xy_all(1,i),xy_all(2,i)] = obj.xy(t(i));
            end
            plot(xy_all(1,:),xy_all(2,:),'LineWidth',2);
        end
        
        function plot_k(obj)
            t = 0:0.01:2*pi;
            k_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                k_all(i) = obj.k(t(i));
            end
            plot(t,k_all,'LineWidth',2);
        end
        
        function plot_k_dot(obj)
            t = 0:0.01:2*pi;
            k_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                k_all(i) = obj.k_dot(t(i));
            end
            plot(t,k_all,'LineWidth',2);
        end
        
        function plot_theta(obj)
            t = 0:0.01:2*pi;
            theta_all = zeros(1,size(t,1));
            for i=1:size(t,2)
                theta_all(i) = obj.theta(t(i));
            end
            plot(t,theta_all,'LineWidth',2);
        end
        
     end
end
