controllers = cellstr(['euler'; 'pid  ']);
trajectories = cellstr(['linear  '; 'circular'; 'squared ']);
axes = cellstr(['x'; 'y']);
measures = cellstr(['iae '; 'ise '; 'itae'; 'itse']);

simulations = containers.Map;
performances = containers.Map;
results = containers.Map;

for i = 1:size(controllers, 1)
    simulations(char(controllers(i))) = containers.Map;
    performances(char(controllers(i))) = containers.Map;

    for j = 1:size(trajectories,1)
        controller = char(controllers(i));
        trajectory = char(trajectories(j));

        temp_controller = simulations(controller);
        temp_controller(trajectory) = ...
            Simulation(controller, trajectory);
        temp_simulation = temp_controller(trajectory);
        
        temp_performance = performances(controller);
        temp_performance(trajectory) = containers.Map;
        temp_axis = temp_performance(trajectory);
        
        for k = 1:size(axes, 1)
            axis = char(axes(k));
            
            t = temp_simulation.data.t;
            
            if strcmp(axis, 'x')
                reference = temp_simulation.data.x_ref;
                actual = temp_simulation.data.x;
            else
                reference = temp_simulation.data.y_ref;
                actual = temp_simulation.data.y;
            end
            
            error = Error(reference, actual);
            
            ise = ISE(error, t);
            iae = IAE(error, t);
            ita = ITAE(error, t);
            itse = ITSE(error, t);

            temp_axis(axis) = Performance(ise, iae, ita, itse);
            temp_performance_axis = temp_axis(axis);
            
            results(strcat(controller(1), trajectory(1), axis, '-', 'iae'))...
                = temp_performance_axis.IAE.compute_integral();
            
            results(strcat(controller(1), trajectory(1), axis, '-', 'ise'))...
                = temp_performance_axis.ISE.compute_integral();
            
            results(strcat(controller(1), trajectory(1), axis, '-', 'itae'))...
                = temp_performance_axis.ITAE.compute_integral();
            
            results(strcat(controller(1), trajectory(1), axis, '-', 'itse'))...
                = temp_performance_axis.ITSE.compute_integral();
        end
    end
end

for i = 1:size(trajectories, 1)
    trajectory = char(trajectories(i));
    
    for j = 1:size(axes, 1)
        axis = char(axes(j));
        
        for k = 1:size(measures, 1)
            measure = char(measures(k));
            fprintf(horzcat('Euler, PID - ', trajectory, ' - ', axis,...
                ' - ', upper(measure), ': %f  %f\n'),...
                results(strcat('e', trajectory(1), axis, '-', measure)),...
                results(strcat('p', trajectory(1), axis, '-', measure)));
        end
        fprintf('\n');
    end
    fprintf('\n\n');
end
