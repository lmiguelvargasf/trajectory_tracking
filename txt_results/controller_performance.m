clear;
clc;

controllers = cellstr(['euler'; 'pid  ']);
trajectories = cellstr(['linear  '; 'circular'; 'squared ']);
axes = cellstr(['x'; 'y']);
measures = cellstr(['iae '; 'ise '; 'itae'; 'itse']);

simulations = containers.Map;
performances = containers.Map;
results = containers.Map;

for i = 1:size(controllers, 1)
    controller = char(controllers(i));
    simulations(controller) = containers.Map;
    performances(controller) = containers.Map;

    for j = 1:size(trajectories,1)
        trajectory = char(trajectories(j));

        temp_sim_controller = simulations(controller);
        temp_sim_controller(trajectory) = ...
            Simulation(controller, trajectory);
        temp_sim_trajectory = temp_sim_controller(trajectory);
        
        temp_perf_controller = performances(controller);
        temp_perf_controller(trajectory) = containers.Map;
        temp_perf_axis = temp_perf_controller(trajectory);

        for k = 1:size(axes, 1)
            axis = char(axes(k));
            
            t = temp_sim_trajectory.data.t;
            
            switch axis
                case 'x'
                    reference = temp_sim_trajectory.data.x_ref;
                    actual = temp_sim_trajectory.data.x;
                case 'y'
                    reference = temp_sim_trajectory.data.y_ref;
                    actual = temp_sim_trajectory.data.y;
            end

            error = Error(reference, actual);

            ise = ISE(error, t);
            iae = IAE(error, t);
            ita = ITAE(error, t);
            itse = ITSE(error, t);

            temp_perf_axis(axis) = Performance(ise, iae, ita, itse);
            temp_performance_axis = temp_perf_axis(axis);
            
            prefix_key = strcat(controller(1), trajectory(1), axis, '-');

            results(strcat(prefix_key, 'iae'))...
                = temp_performance_axis.IAE.compute_integral();
            
            results(strcat(prefix_key, 'ise'))...
                = temp_performance_axis.ISE.compute_integral();
            
            results(strcat(prefix_key, 'itae'))...
                = temp_performance_axis.ITAE.compute_integral();
            
            results(strcat(prefix_key, 'itse'))...
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
