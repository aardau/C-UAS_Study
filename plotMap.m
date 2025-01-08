% This function generates the map and plots any map features generated 
% from the setupMap function

% Inputs:
% mapFeatures: Structure variable that contains the generated map features
%               and their associated information (x-pos,y-pos,range)
% xmin: Minimum value of the map's x-direction length from zero
% xmax: Maximum value of the map's x-direction length from zero
% ymin: Minimum value of the map's y-direction length from zero
% ymax: Maximum value of the map's y-direction length from zero
% xposUAS: Vector containing x-positions of UAS
% yposUAS: Vector containing y-positions of UAS
% fixedPoint: Fixed point where UAS spawns

% Outputs:
% Figure of plotted map with map features and UAS

function plotMap(mapData, xmin, xmax, ymin, ymax, xposUAS, yposUAS, fixedPoint)
    
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
    pgon = polyshape(mapData.base.x, mapData.base.y);
    plot(pgon);

    % Plot UAS path on the map
    plot(xposUAS, yposUAS, 'r', 'LineWidth', 2);
    hold on;
    plot(fixedPoint(1), fixedPoint(2), 'bs', 'MarkerSize', 10);

    % Adjust plot settings
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('UAS Simulation');
    %legend
    grid on;
end