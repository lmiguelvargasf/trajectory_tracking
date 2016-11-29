classdef (Abstract) PerformanceMeasurement
   properties
       error
       time
   end
   methods (Abstract)
      compute_integral(obj)   
   end
end
