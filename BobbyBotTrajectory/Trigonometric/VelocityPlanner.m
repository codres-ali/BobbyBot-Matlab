classdef VelocityPlanner < handle
    properties
         b2 = 0;
         tf = 0;
         vf = 0;
         vi = 0;
         a_max = 0;
         s_max = 0;
    end
    
    methods
        function obj=VelocityPlanner(varargin)
            var_ind = 1;
            while var_ind <= nargin
                switch varargin{var_ind}
                    case 'vf'
                        obj.vf = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'vi'
                        obj.vi = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'a_max'
                        obj.a_max = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    case 'tf'
                        obj.tf = varargin{var_ind + 1};
                        var_ind = var_ind + 2;  % index to next variable
                    otherwise
                        error(['Unexpected option: `' varargin{var_ind} '`!'])
                end
            end
            
            if obj.tf == 0
                if obj.a_max~=0
                    obj.tf = 4*((obj.vf+obj.vi)/2-obj.vi)/obj.a_max;
                    if obj.tf<0
                        obj.tf = -obj.tf;
                        obj.a_max = -obj.a_max;
                    end
                    obj.b2 = ((obj.vf+obj.vi)/2-obj.vi)/(obj.tf/2)^2;
                end
            else
                obj.b2 = ((obj.vf+obj.vi)/2-obj.vi)/(obj.tf/2)^2;
                obj.a_max = obj.b2*obj.tf;
            end
            
            obj.s_max = obj.s(obj.tf);
        end
        
        function out=v(obj,t)  
            out = 0;
            if t>=0 && t<obj.tf/2
                out = obj.b2*t^2+obj.vi;
            else
                if t>=obj.tf/2 && t<=obj.tf
                    out = -obj.b2*(t-obj.tf)^2+obj.vf;
                end
            end
        end
        
        function out=a(obj,t)  
            out = 0;
            if t>0 && t<obj.tf/2
                out = 2*obj.b2*t;
            else
                if t>=obj.tf/2 && t<=obj.tf
                    out = -2*obj.b2*(t-obj.tf);
                end
            end
        end
        
        function out=s(obj,t)  
            out = 0;
            if t>=0 && t<obj.tf/2
                out = 1/3*obj.b2*t^3+obj.vi*t;
            else
                if t>=obj.tf/2 && t<=obj.tf
                    out = 1/3*obj.b2*(obj.tf-t)^3+obj.vf*(t-obj.tf/2)+obj.vi*obj.tf/2;
                end
            end
        end
        
        function plot_v(obj)
            t = 0:0.01:obj.tf-0.01;
            v_all = zeros(1,size(t,1));
            for k=1:size(t,2)
                v_all(k) = obj.v(t(k));
            end
            plot(t,v_all);
        end
        
        function plot_a(obj)
            t = 0:0.01:obj.tf-0.01;
            a_all = zeros(1,size(t,1));
            for k=1:size(t,2)
                a_all(k) = obj.v_dot(t(k));
            end
            plot(t,a_all);
        end
        
        function plot_s(obj)
            t = 0:0.01:obj.tf;
            s_all = zeros(1,size(t,1));
            for k=1:size(t,2)
                s_all(k) = obj.s(t(k));
            end
            plot(t,s_all);
        end
     end
end
