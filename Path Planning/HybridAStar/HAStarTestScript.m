clc; clear; close all
tic
% Define map size
mapSize = [5000, 5000];
mapData = zeros(mapSize(1), mapSize(2)); % Empty occupancy grid

% Define multiple polygon obstacles as cell arrays of [x, y] coordinates
polygons = {
    [1000 4000; 1500 3500; 1500 3000; 1000 2500];
    [750 1250; 3250 1250; 3250 1000; 750 1000];
    [3250 4500; 3250 3500; 3750 2000; 3750 4000];
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
% Define map boundaries
xmin = 1;
xmax = mapSize(2);
ymin = 1;
ymax = mapSize(1);

goalPose = [2500,2500, 0];

% Choose a random edge of the map to spawn the UAS
edge = randi(4);
switch edge
    case 1 % Top edge
        startPose = [randi([xmin, xmax]), ymax, 3*pi/2];
    case 2 % Bottom edge
        startPose = [randi([xmin, xmax]), ymin, pi/2];
    case 3 % Left edge
        startPose = [xmin, randi([ymin, ymax]), 0];
    case 4 % Right edge
        startPose = [xmax, randi([ymin, ymax]), pi];
    otherwise % Prints warning if no case is satisfied
        warning('Error in UAS spawning from switch cases')
end

% Keep the goal's heading the same as the start pose
goalPose(3) = startPose(3);


% Plan the path for the specified start pose, and goal pose, and map
path = HAStarTestFunc(mapData,startPose,goalPose);

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

toc