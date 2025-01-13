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

function plotMap(mapFeatures, bounds, xposUAS, yposUAS)
    
    % Extract map bounds for calculations
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    
    % Plot everything in one figure
    figure;
    hold on;

    % Plot static defenses
    for i = 1:height(mapFeatures.staticDefenses)
        x = mapFeatures.staticDefenses(i, 1) - mapFeatures.staticDefenses(i, 3):1:mapFeatures.staticDefenses(i, 1) + mapFeatures.staticDefenses(i, 3);
        R = mapFeatures.staticDefenses(i, 3);
        y1 = sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
        y2 = -sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
        plot(x, y1, x, y2,'Color',"r");
    end

    %Plot mobile defenses
    for j = 1:height(mapFeatures.mobileDefenses)
        x = mapFeatures.mobileDefenses(j, 1) - mapFeatures.mobileDefenses(j, 3):1:mapFeatures.mobileDefenses(j, 1) + mapFeatures.mobileDefenses(j, 3);
        R = mapFeatures.mobileDefenses(j, 3);
        y1 = sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
        y2 = -sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
        plot(x, y1, x, y2,'Color',"b");        
    end

    % Plot base
    base = polyshape(mapFeatures.base.x, mapFeatures.base.y);
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