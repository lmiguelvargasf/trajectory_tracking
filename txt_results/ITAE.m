classdef ITAE < PerformanceMeasurement 
    
    properties
    end

    methods
        function obj = ITAE(error, time)
            obj.error = error;
            obj.time = time;
        end
        
        
        function value = compute_integral(obj)
            value = trapz(obj.time, obj.time .* abs(obj.error.error));
        end
    end
    
end
