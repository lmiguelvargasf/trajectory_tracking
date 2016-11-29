classdef Error
   properties
       error
   end
   methods
       function obj = Error(reference, actual)
           obj.error = reference - actual;
       end
   end
end
