% This function generates the map given the structures 
% Inputs:
% -mapData: Structure variable that contains the generated structures and
%           their associated information
% -xmin: Minimum value of the map's x-direction length from zero
% -xmax: Maximum value of the map's x-direction length from zero
% -ymin: Minimum value of the map's y-direction length from zero
% -ymax: Maximum value of the map's y-direction length from zero
% Outputs:
% -Plot of map with structures

function plotMap(mapData, xmin, xmax, ymin, ymax)
    % Plot the map using data from setupMap
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

    % Adjust plot settings
    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Generated Map');
    grid on;
end