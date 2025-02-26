% Test Hyrbid A* with trajectory map
clc;clear;close all

mapSize = [50, 50]; % 50m x 50m map
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
goalPose = [45 45 pi/2];

% Define kinematic restraints
maxVel = 1; % UAV speed in m/s
maxTurnAngleDeg = 45; % Maximum steering angle

% Define timestep
dt = 1; % Timestep (1 second)

% Call function
trajectory = HAstarTrajectory(mapData, startPose, goalPose, maxVel, maxTurnAngleDeg, dt);

% Plot the trajectory
figure; hold on;
show(binaryOccupancyMap(mapData));
scatter(startPose(1), startPose(2), 'g', 'filled'); % Start point
scatter(goalPose(1), goalPose(2), 'r', 'filled'); % Goal point
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2); % UAV path
legend("Start", "Goal", "UAV Trajectory");