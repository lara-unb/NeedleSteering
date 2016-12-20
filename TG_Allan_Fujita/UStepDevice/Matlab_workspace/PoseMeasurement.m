classdef PoseMeasurement < hgsetget
    
    %
    %   This class actually works like a struct
    %
    
    % Author: André Augusto Geraldes
    % Email: andregeraldes@lara.unb.br
    % August 2015; Last revision:
    
    properties (GetAccess = public, SetAccess = public)
        x;       
        y;
        z;
        orientation;
        error;
    end
    
    methods (Access = public)
        
        function obj = PoseMeasurement()
            obj.x = 0.0;
            obj.y = 0.0;
            obj.z = 0.0;
            obj.orientation = [0.0 0.0 0.0 0.0];
            obj.error = 0.0;            
        end
        
    end
    
end