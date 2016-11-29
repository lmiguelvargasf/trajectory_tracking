classdef Performance

    properties
        ISE
        IAE
        ITAE
        ITSE
    end
    
    methods
        function obj = Performance(ISE, IAE, ITAE, ITSE)
           obj.ISE = ISE;
           obj.IAE = IAE;
           obj.ITAE = ITAE;
           obj.ITSE = ITSE;
       end
    end
    
end
