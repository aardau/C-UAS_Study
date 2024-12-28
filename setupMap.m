% This function creates the various structures and environmental details
% used in the map generation in the plotMap function
% Inputs:
% -xmin: Minimum value of the map's x-direction length from zero
% -xmax: Maximum value of the map's x-direction length from zero
% -ymin: Minimum value of the map's y-direction length from zero
% -ymax: Maximum value of the map's y-direction length from zero
% -rangemin: Minimum effector detection radius
% -rangemax: Maximum effector detection radius
% -numEffectors: number of effectors that spawn on the map
% Outputs:
% -mapData: Structure variable that contains the generated structures and
%           their associated information

function mapData = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors)
    % Setup effectors
    mapData.effectors = zeros(numEffectors, 3); %Initialize
    for i = 1:numEffectors
        mapData.effectors(i, 1) = randi([xmin, xmax]); % X coordinates
        mapData.effectors(i, 2) = randi([ymin, ymax]); % Y coordinates
        mapData.effectors(i, 3) = randi([rangemin, rangemax]); % Range
    end
    
    % Spawn base at origin
    mapData.base = struct('x', [-100, 100, 100, -100], 'y', [-100, -100, 100, 100]);

    % Additional map features, like no-fly zones, can be added here 
end
