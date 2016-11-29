classdef SimulationData
    
    properties
        t
        x
        y
        x_ref
        y_ref
    end
    
    methods
        function obj = SimulationData(controller, trajectory)
            files = cellstr(['t    '; 'x    '; 'y    '; 'x_ref'; 'y_ref']);
            for i = 1:size(files,1)
                file = char(files(i));
                path = strcat('./', controller, '/',...
                    trajectory, '/', file, '.txt');
             
                fileID = fopen(path,'r');
                formatSpec = '%f';

                 switch file
                     case 't'
                         obj.t = fscanf(fileID, formatSpec);
                     case 'x'
                         obj.x = fscanf(fileID, formatSpec);
                     case 'y'
                         obj.y = fscanf(fileID, formatSpec);
                     case 'x_ref'
                         obj.x_ref = fscanf(fileID, formatSpec);
                     case 'y_ref'
                         obj.y_ref = fscanf(fileID, formatSpec);
                 end
                 fclose(fileID);
            end
        end
    end
    
end

