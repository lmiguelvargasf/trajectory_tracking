classdef ISE < PerformanceMeasurement 
    
    properties
    end

    methods
        function obj = ISE(error, time)
            obj.error = error;
            obj.time = time;
        end
        
        function value = compute_integral(obj)
            value = trapz(obj.time, obj.error.error .^ 2);
        end
    end
    
end
