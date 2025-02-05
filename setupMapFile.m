% This function creates the various map features used in the map generation

% Inputs:
% bounds: Maximum and minimum map bounds measured from zero in the form 
%         [xmin, xmax, ymin, ymax]
% rangBounds: Maximum and minimum range for defenses
% numDefenses: number of defenses that spawn on the map in the form
%              [numStaticDefenses, numMobileDefenses]

% Outputs:
% mapFeatures: Structure variable that contains the generated map features
%               and their associated information (x-pos,y-pos,range)

%% ALL ABOVE NEEDS REWRITTEN

function mapFeatures = setupMapFile(mapBounds, effectorData, limits)
    
    % Extract map bounds for calculations
    xmin = mapBounds(1); xmax = mapBounds(2);
    ymin = mapBounds(3); ymax = mapBounds(4);

    rangeMin = limits(1, 1);
    rangeMax = limits(1, 2);
    
    for i=1:height(effectorData)
        % randomly generate non-set parameters
        if isnan(effectorData(i, 3))
            effectorData(i, 3) = randi([rangeMin, rangeMax]);
        end
        placementX = [xmin+effectorData(i, 3), xmax-effectorData(i, 3)];
        placementY = [ymin+effectorData(i, 3), ymax-effectorData(i, 3)];
        if isnan(effectorData(i, 1))
            effectorData(i, 1) = randi(placementX);
        end
        if isnan(effectorData(i, 2))
            effectorData(i, 2) = randi(placementY);
        end
        if isnan(effectorData(i, 4))
            effectorData(i, 4) = randi([0,1]);
        end
    end

    % calculate static vs. mobile number
    numMobileDefenses = sum(effectorData(:, 4), 'omitnan'); % Ignore NaN values
    numStaticDefenses = height(effectorData) - numMobileDefenses;

    % create indexes for static + mobile defenses
    si = 1;
    mi = 1;

    % initialize static + mobile data vectors
    mapFeatures.staticDefenses = zeros(numStaticDefenses, 3); 
    mapFeatures.mobileDefenses = zeros(numMobileDefenses, 3);
    % iterate through data file
    for i = 1:height(effectorData)
        % determine if line is mobile / static + assign
        if effectorData(i, 4) == 0
            mapFeatures.staticDefenses(si, :) = effectorData(i, 1:3);
            si = si+1;
        end
        if effectorData(i, 4) == 1
            mapFeatures.mobileDefenses(mi, :) = effectorData(i, 1:3);
            mi = mi+1;
        end
    end

    % Spawn base at origin
    mapFeatures.base = struct('x', [-500, 500, 500, -500], 'y', [-500, -500, 500, 500]);

    % Define obstacles for the Hybrid A* path planning model to path around
    mapFeatures.obstacles.number = 3;
    mapFeatures.obstacles.type = 1; % 0 = AABB, 1 = Polygon --> Keep as 1 
    mapFeatures.obstacles.vertices{1} = [-1500,1500; -1000,1000; -1000,500; -1500,0];
    mapFeatures.obstacles.vertices{2} = [-1750,-1250; 750,-1250; 750,-1500; -1750,-1500]; 
    mapFeatures.obstacles.vertices{3} = [750,2000; 750,1000; 1250,-500; 1250,1500];
    
    
    % Additional map features, like no-fly zones and terrain, can be added 
    % here. Don't forget to write it as a structure var!
end
