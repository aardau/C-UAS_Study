% This function generates the map and plots any map features generated 
% from the setupMap function

% Inputs:
% mapFeatures: Structure variable that contains the generated map features
%               and their associated information (x-pos,y-pos,range)
% bounds: Maximum and minimum map bounds measured from zero in the form 
%         [xmin, xmax, ymin, ymax]
% xposUAS: Vector containing x-positions of UAS
% yposUAS: Vector containing y-positions of UAS
% fixedPoint: Fixed point where UAS spawns

% Outputs:
% Figure of plotted map with map features and UAS

function plotMap(mapData, bounds, xposUAS, yposUAS)
    
    % Extract map bounds for calculations
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    
    % Plot everything in one figure
    figure;
    hold on;

    % Plot effectors
    for i = 1:size(mapData.effectors, 1)
        x = mapData.effectors(i, 1) - mapData.effectors(i, 3):1:mapData.effectors(i, 1) + mapData.effectors(i, 3);
        R = mapData.effectors(i, 3);
        y1 = sqrt(R^2 - (x - mapData.effectors(i, 1)).^2) + mapData.effectors(i, 2);
        y2 = -sqrt(R^2 - (x - mapData.effectors(i, 1)).^2) + mapData.effectors(i, 2);
        plot(x, y1, x, y2);
    end

    % Plot base
    base = polyshape(mapData.base.x, mapData.base.y);
    plot(base);

    % Plot UAS path
    plot(xposUAS, yposUAS, 'r', 'LineWidth', 2);

    % Adjust plot settings
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('UAS Simulation');
    %legend
    grid on;
end