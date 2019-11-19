classdef VISIR
    %Orbital propagator 
    properties
        % Earth Properties
        Re = 6378.1e3;          % Radius [m]
        mu_e = 398600.436233;   % Gravitational Parameter [km^3/s^2]
        
        % Moon
        Rm = 1;                 % Radius [m]
        mu_m = 4902.800076;     % Gravitational Parameter [km^3/s^2]
    end
    
    methods
        function obj = untitled2(inputArg1,inputArg2)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

