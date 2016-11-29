classdef ITSE < PerformanceMeasurement 
    
    properties
    end

    methods
        function obj = ITSE(error, time)
            obj.error = error;
            obj.time = time;
        end
        
        function value = compute_integral(obj)
            value = trapz(obj.time, obj.time .* obj.error.error .^ 2);
        end
    end
    
end
