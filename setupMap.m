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

function mapFeatures = setupMap(mapBounds, defenseRanges, numDefenses)
    
    % Extract map bounds for calculations
    xmin = mapBounds(1); xmax = mapBounds(2);
    ymin = mapBounds(3); ymax = mapBounds(4);

    %Extract range bounds for calculations
    rangeminStatic = defenseRanges(1); rangemaxStatic = defenseRanges(2);
    rangeminMobile = defenseRanges(3); rangemaxMobile = defenseRanges(4);

    % Setup static defenses
    mapFeatures.staticDefenses = zeros(numDefenses(1), 3); %Initialize
    for i = 1:height(mapFeatures.staticDefenses)
        mapFeatures.staticDefenses(i, 1) = randi([xmin, xmax]); % X coordinate(s)
        mapFeatures.staticDefenses(i, 2) = randi([ymin, ymax]); % Y coordinate(s)
        mapFeatures.staticDefenses(i, 3) = randi([rangeminStatic, rangemaxStatic]); % Range
    end
    
    % Setup mobile defenses
    mapFeatures.mobileDefenses = zeros(numDefenses(2),3); %Initialize
    for j = 1:height(mapFeatures.mobileDefenses)
        mapFeatures.mobileDefenses(j, 1) = randi([xmin, xmax]); % X coordinate(s)
        mapFeatures.mobileDefenses(j, 2) = randi([ymin, ymax]); % Y coordinate(s)
        mapFeatures.mobileDefenses(j, 3) = randi([rangeminMobile, rangemaxMobile]); % Range
    end

    % Spawn base at origin
    mapFeatures.base = struct('x', [-100, 100, 100, -100], 'y', [-100, -100, 100, 100]);

    % Additional map features, like no-fly zones and terrain, can be added 
    % here. Don't forget to write it as a structure var!
end
