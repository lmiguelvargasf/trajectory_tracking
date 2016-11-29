classdef Simulation

    properties
        controller
        trajectory
        data
    end
    
    methods
        function obj = Simulation(controller, trajectory)
           obj.controller = controller;
           obj.trajectory = trajectory;
           obj.data = SimulationData(controller, trajectory);
        end
    end
end


