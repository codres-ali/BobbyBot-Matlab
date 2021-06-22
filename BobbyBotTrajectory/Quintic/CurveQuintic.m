classdef CurveQuintic < handle
    properties
        P0;
        P1;
        A;
        B;
        eta = 1;
        s = [];
        s_max = 0;
        dt = 0.001;
    end
    
    methods
        function obj=CurveQuintic(varargin)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'P0'
                        obj.P0 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'P1'
                        obj.P1 = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'eta'
                        obj.eta = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'dt'
                        obj.dt = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            
            eta1 = obj.eta;
            eta2 = obj.eta;

            obj.A(1) = obj.P0(1);
            obj.A(2) = eta1*cos(obj.P0(3));
            obj.A(3) = 0;
            obj.A(4) = 10*(obj.P1(1)-obj.P0(1))-6*eta1*cos(obj.P0(3))-4*eta2*cos(obj.P1(3));
            obj.A(5) = -15*(obj.P1(1)-obj.P0(1))+8*eta1*cos(obj.P0(3))+7*eta2*cos(obj.P1(3));
            obj.A(6) = 6*(obj.P1(1)-obj.P0(1))-3*eta1*cos(obj.P0(3))-3*eta2*cos(obj.P1(3));

            obj.B(1) = obj.P0(2);
            obj.B(2) = eta1*sin(obj.P0(3));
            obj.B(3) = 0;
            obj.B(4) = 10*(obj.P1(2)-obj.P0(2))-6*eta1*sin(obj.P0(3))-4*eta2*sin(obj.P1(3));
            obj.B(5) = -15*(obj.P1(2)-obj.P0(2))+8*eta1*sin(obj.P0(3))+7*eta2*sin(obj.P1(3));
            obj.B(6) = 6*(obj.P1(2)-obj.P0(2))-3*eta1*sin(obj.P0(3))-3*eta2*sin(obj.P1(3));
            
            t = 0:obj.dt:1+obj.dt;
            obj.s = zeros(1,size(t,2));
            [x_dot,y_dot] = obj.xy_dot(t(1));
            dsk = (x_dot^2 + y_dot^2)^0.5;
            for k=2:size(t,2)
                dsk_1 = dsk;
                [x_dot,y_dot] = obj.xy_dot(t(k));
                dsk = (x_dot^2 + y_dot^2)^0.5;
                obj.s(k) = obj.s(k-1)+obj.dt*(dsk+dsk_1)/2;
            end
            obj.s_max = obj.s_arc(1);
        end
        
        function [x,y]=xy(obj,t)  
            x = 0;
            y = 0;
            if t>=0 && t<=1
                x = obj.A(1)+obj.A(2)*t+obj.A(3)*t^2+obj.A(4)*t^3+obj.A(5)*t^4+obj.A(6)*t^5;
                y = obj.B(1)+obj.B(2)*t+obj.B(3)*t^2+obj.B(4)*t^3+obj.B(5)*t^4+obj.B(6)*t^5;
            end
        end
        
        function [x_dot,y_dot]=xy_dot(obj,t)  
            x_dot = 0;
            y_dot = 0;
            if t>=0 && t<=1
                x_dot = obj.A(2)+2*obj.A(3)*t+3*obj.A(4)*t^2+4*obj.A(5)*t^3+5*obj.A(6)*t^4;
                y_dot = obj.B(2)+2*obj.B(3)*t+3*obj.B(4)*t^2+4*obj.B(5)*t^3+5*obj.B(6)*t^4;
            end
        end
        
        function [x_dot_dot,y_dot_dot]=xy_dot_dot(obj,t)  
            x_dot_dot = 0;
            y_dot_dot = 0;
            if t>=0 && t<=1
                x_dot_dot = 2*obj.A(3)+6*obj.A(4)*t+12*obj.A(5)*t^2+20*obj.A(6)*t^3;
                y_dot_dot = 2*obj.B(3)+6*obj.B(4)*t+12*obj.B(5)*t^2+20*obj.B(6)*t^3;
            end
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
            if t>=0 && t<=1
                [x_dot, y_dot] = obj.xy_dot(t);
                [x_dot_dot, y_dot_dot] = obj.xy_dot_dot(t);
                k = (x_dot*y_dot_dot-x_dot_dot*y_dot)/(x_dot^2+y_dot^2)^1.5;
            end
        end
        
        function k_dot=k_dot(obj,t)  
            k_dot = 0;
            if t>=0 && t<=2*pi
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
            t = 0:0.01:1;
            xy_all = zeros(2,size(t,1));
            for i=1:size(t,2)
                [xy_all(1,i),xy_all(2,i)] = obj.xy(t(i));
            end
            plot(xy_all(1,:),xy_all(2,:),'r','LineWidth',2);
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
