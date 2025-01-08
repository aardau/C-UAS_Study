% This function creates the various map features used in the map generation

% Inputs:
% xmin: Minimum value of the map's x-direction length from zero
% xmax: Maximum value of the map's x-direction length from zero
% ymin: Minimum value of the map's y-direction length from zero
% ymax: Maximum value of the map's y-direction length from zero
% rangemin: Minimum effector detection radius
% rangemax: Maximum effector detection radius
% numEffectors: number of effectors that spawn on the map

% Outputs:
% mapFeatures: Structure variable that contains the generated map features
%               and their associated information (x-pos,y-pos,range)

function mapFeatures = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors)
    
    % Setup effectors
    mapFeatures.effectors = zeros(numEffectors, 3); %Initialize
    for i = 1:numEffectors
        mapFeatures.effectors(i, 1) = randi([xmin, xmax]); % X coordinates
        mapFeatures.effectors(i, 2) = randi([ymin, ymax]); % Y coordinates
        mapFeatures.effectors(i, 3) = randi([rangemin, rangemax]); % Range
    end
    
    % Spawn base at origin
    mapFeatures.base = struct('x', [-100, 100, 100, -100], 'y', [-100, -100, 100, 100]);

    % Additional map features, like no-fly zones and terrain, can be added 
    % here 
end
