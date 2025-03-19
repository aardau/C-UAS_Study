%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Effector generation parameters
rangeMin = 500;
rangeMax = 1000;
mobileDefenseSpeed = 10;

% Create limits matrix
limits = [rangeMin, rangeMax;];

% Kill-chain probabilities
trackProb = 0.1;
killProb = 0.5;

% Specify file name for defense placements
%fn = "effector_inputs.xlsx"; % all static, lots of randomization
fn = "effector_inputs_SPECIFIED.xlsx"; % entirely specified starting condition, 3S 2M
%fn = "effector_inputs_RAND.xlsx"; % entirely randomized setup
effectorData = readmatrix(fn);

% UAS parameters
velUAS = 20; % Velocity (units/s)
minTurnRad = 10; % Minimum turn radius (units)
dT = 1; % Time step (s) (don't change)

%% Generate map features
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

% Plot static map features that are not affected by Monte-Carlo
figure(1);
hold on
% Plot base
base = polyshape(mapFeatures.base.x, mapFeatures.base.y);
plot(base);

% Plot map obstacles
obstacles = mapFeatures.obstacles;  % Extract obstacles from mapFeatures
xO = []; % Initialize X vals
yO = []; % Initialize y vals

if isfield(mapFeatures, 'obstacles') && mapFeatures.obstacles.number > 0
    for k = 1:obstacles.number
        xO = [xO; obstacles.vertices{k}(:,1); obstacles.vertices{k}(1,1); NaN];
        yO = [yO; obstacles.vertices{k}(:,2); obstacles.vertices{k}(1,2); NaN];
        fill(obstacles.vertices{k}(:,1), obstacles.vertices{k}(:,2), 'k'); % Fill obstacles in black
    end
    plot(xO, yO, 'Color', [0.2, 0.2, 1], 'LineWidth', 2);
end

%% Start Monte-Carlo

% Define parameters for Monte Carlo analysis
maxIterations = 500;  % Set an upper limit in case convergence does not happen
killVar = zeros(maxIterations, 1);  % Initialize
killXY = NaN(maxIterations, 2);  % Initialize
windowSize = 100;  % Number of iterations to check for convergence

% Store success rate history
successRateHistory = zeros(maxIterations, 1);
stdSuccessRate = zeros(maxIterations, 1);  % Initialize standard deviation storage

% Initialize success rate plot
figure(2);
successPlot = plot(nan, nan, 'b-', 'LineWidth', 2); % Empty plot
xlabel('Number of Simulations');
ylabel('Defense Success Rate (%)');
title('Monte Carlo Defense Success Rate Convergence');
grid on;
hold on;

% Initialize standard deviation plot
figure(3);
stdPlot = plot(nan, nan, 'r-', 'LineWidth', 2); % Empty plot
xlabel('Number of Simulations');
ylabel('Standard Deviation of Success Rate (%)');
title('Standard Deviation of Defense Success Rate Over Time');
grid on;

% Randomize randi seed for this session
rng('shuffle')

% Start Monte Carlo loop
for N = 1:maxIterations
%% Hybrid A*
% Generate UAS path using a Hybrid A* path planning algorithm

maxAttempts = 5; % Limit retries to prevent infinite loop
attempts = 0;
uasPath = [];

while isempty(uasPath) && attempts < maxAttempts
    fprintf('Path generation attempt %d...\n', attempts + 1);
    uasPath = NEW_hybridAStarFunc(mapBounds, mapFeatures, velUAS, minTurnRad, dT);
    attempts = attempts + 1;
end

if isempty(uasPath)
    error('uasPath is empty! Check why NEW_hybridAStarFunc is not generating a path.');
end

% Extract the (x,y) coordinates from uasPath for use in other functions
uasPosition = [uasPath(:,1), uasPath(:,2)];

%% Mobile Defense Movement
% select closest defense
if height(mapFeatures.mobileDefenses) > 0
selectedMobileDefense = mobileDefenseSelection(height(mapFeatures.mobileDefenses), mapFeatures.mobileDefenses, uasPosition);
mDInitialPosition = mapFeatures.mobileDefenses(selectedMobileDefense, 1:2);
mobileDefensePosition = mobileDefensePathing(uasPosition, mDInitialPosition, mobileDefenseSpeed, dT);
else
    mobileDefensePosition = [NaN, NaN];
    selectedMobileDefense = NaN;
end

%% Kill Chain
%returns new terminated flight tracks and positions of kill
%[updatedAdversaryPosition, killPoints] = killChain(uasPosition, mapFeatures);
[SDHits, MDHits] = killDetection(mapFeatures, uasPosition, mobileDefensePosition, selectedMobileDefense);
updatedAdversaryPosition = uasPosition;
[killVar(N), ~, killXY(N, :)] = killCheck(SDHits, MDHits, trackProb, killProb, mapFeatures, uasPosition);

%% Calculate and plot success rate

defenseRate = sum(killVar(1:N)) / N * 100;
successRateHistory(N) = defenseRate;

% Compute standard deviation based on window size
if N > windowSize
    stdSuccessRate(N) = std(successRateHistory(N-windowSize+1:N));
else
    stdSuccessRate(N) = std(successRateHistory(1:N));
end

% Update success rate plot
set(successPlot, 'XData', 1:N, 'YData', successRateHistory(1:N));

% Update standard deviation plot
set(stdPlot, 'XData', 1:N, 'YData', stdSuccessRate(1:N));

% Refresh Plots
drawnow;

%% Plot others
% Plot the map, map features, and UAS track
 plotMap(mapFeatures, mapBounds, updatedAdversaryPosition, mobileDefensePosition);

% Plot mobile defense movement
x = mobileDefensePosition(:, 1); y = mobileDefensePosition(:, 2);
plot(x, y, 'cx')
if killVar(N) == 1
    plot(killXY(N, 1), killXY(N, 2), 'gx', 'MarkerSize', 30, 'LineWidth', 8)
end

end % End of Monte-Carlo loop