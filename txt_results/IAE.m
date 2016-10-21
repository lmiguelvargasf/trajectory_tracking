classdef IAE < PerformanceMeasurement 
    
    properties
    end

    methods
        function obj = IAE(error, time)
            obj.error = error;
            obj.time = time;
        end
        
        function value = compute_integral(obj)
            value = trapz(obj.time, abs(obj.error.error));
        end
    end
    
end
