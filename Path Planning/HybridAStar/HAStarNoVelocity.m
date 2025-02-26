clc; clear; close all

% Define map size
mapSize = [50, 50];
mapData = zeros(mapSize(1), mapSize(2)); % Empty occupancy grid

% Define multiple polygon obstacles as cell arrays of [x, y] coordinates
polygons = {
    [15 10; 30 15; 25 35; 10 30];
    [35 10; 45 10; 45 20; 35 20];
    [20 40; 30 40; 28 48; 18 45]
};

% Generate a grid of x, y coordinates for the map
[X, Y] = meshgrid(1:mapSize(2), 1:mapSize(1));

% Loop through each polygon and mark it in the occupancy grid
for i = 1:length(polygons)
    % Extract X and Y coordinates
    polyX = polygons{i}(:,1);
    polyY = polygons{i}(:,2);
    
    % Check which grid points are inside the polygon
    insideObstacle = inpolygon(X, Y, polyX, polyY);
    
    % Assign occupied values to the binary occupancy grid
    mapData(insideObstacle) = 1;
end

% Define start and goal poses as [x y theta] vectors. x and y specify the 
% position in meters, and theta specifies the orientation angle in radians
startPose = [3 3 pi/2];
goalPose = [43 45 pi/2];

% Plan the path for the specified start pose, and goal pose, and map
path = codegenPathPlanner(mapData,startPose,goalPose);

% Visualize the computed path.
show(binaryOccupancyMap(mapData))
hold on
% Start state
scatter(startPose(1,1),startPose(1,2),"g","filled")
% Goal state
scatter(goalPose(1,1),goalPose(1,2),"r","filled")
% Path
plot(path(:,1),path(:,2),"r-",LineWidth=2)
legend("Start Pose","Goal Pose","MATLAB Generated Path")
legend(Location="northwest")