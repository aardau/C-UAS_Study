
function mapData = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors)
    % Setup effector data
    mapData.effectors = zeros(numEffectors, 3);
    for i = 1:numEffectors
        mapData.effectors(i, 1) = randi([xmin, xmax]); % X coordinate
        mapData.effectors(i, 2) = randi([ymin, ymax]); % Y coordinate
        mapData.effectors(i, 3) = randi([rangemin, rangemax]); % Range
    end
    
    % Spawn base
    mapData.base = struct('x', [-100, 100, 100, -100], 'y', [-100, -100, 100, 100]);

    % Additional map features can be added here 
end
