classdef PIDControl < handle
    properties
         
        Kp = 0.05;
        Ti = 0.05;
        Td = 0;
        err_int,der,err,u;
    end
    
    methods
        function obj=PIDControl()
            obj.err=0;
            obj.err_int=0;
            obj.der = 0;
            obj.u = 0;
        end
        function obj=Control(obj,r,y,dt)
            er = r-y;
            obj.der = (er-obj.err)/dt;
            obj.err_int = obj.err_int+er*dt;
            obj.u = obj.Kp*(er+1/obj.Ti*obj.err_int+obj.Td*obj.der);
            obj.err = er;
        end
    end
    
end

