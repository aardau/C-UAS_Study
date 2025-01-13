% This function creates the various map features used in the map generation

% Inputs:
% bounds: Maximum and minimum map bounds measured from zero in the form 
%         [xmin, xmax, ymin, ymax]
% rangemin: Minimum effector detection radius
% rangemax: Maximum effector detection radius
% numEffectors: number of effectors that spawn on the map

% Outputs:
% mapFeatures: Structure variable that contains the generated map features
%               and their associated information (x-pos,y-pos,range)

function mapFeatures = setupMap(bounds, rangemin, rangemax, numEffectors)
    
    % Extract map bounds for calculations
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);

    % Setup static defenses
    mapFeatures.staticDefenses = zeros(numEffectors, 3); %Initialize
    for i = 1:numEffectors
        mapFeatures.staticDefenses(i, 1) = randi([xmin, xmax]); % X coordinates
        mapFeatures.staticDefenses(i, 2) = randi([ymin, ymax]); % Y coordinates
        mapFeatures.staticDefenses(i, 3) = randi([rangemin, rangemax]); % Range
    end
    
    % Setup mobile defenses
    %mapFeatures.mobileDefenses = 

    % Spawn base at origin
    mapFeatures.base = struct('x', [-100, 100, 100, -100], 'y', [-100, -100, 100, 100]);

    % Additional map features, like no-fly zones and terrain, can be added 
    % here. Don't forget to write it as a structure var!
end
