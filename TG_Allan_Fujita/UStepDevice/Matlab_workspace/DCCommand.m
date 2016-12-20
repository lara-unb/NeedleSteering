classdef DCCommand < handle
    
    %
    %   This class actually works like a struct
    %
    
    % Author: André Augusto Geraldes
    % Email: andregeraldes@lara.unb.br
    % August 2015; Last revision:
    
    properties (GetAccess = public, SetAccess = public)
        step_size;       
        insertion_speed;
        rotation_speed;
        duty_cycle;
    end
    
    methods (Access = public)
        
        function obj = DCCommand()
            obj.step_size = 0.0;
            obj.insertion_speed = 0.0;
            obj.rotation_speed = 0.0;
            obj.duty_cycle = 0.0;            
        end
        
    end
    
end