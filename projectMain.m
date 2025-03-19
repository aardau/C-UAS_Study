%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Effector generation parameters
rangeMin = 500;
rangeMax = 500;
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
minTurnRad = 8; % Minimum turn radius (units)
dT = 1; % Time step (s) (don't change)

%% Generate map features 
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

%% Plot run-independent map features once
figure(1);
hold on

% Plot base once
base = polyshape(mapFeatures.base.x, mapFeatures.base.y);
plot(base);

% Plot map obstacles once
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

%Plot static defenses once
for i = 1:height(mapFeatures.staticDefenses)
    x = mapFeatures.staticDefenses(i, 1) - mapFeatures.staticDefenses(i, 3):1:mapFeatures.staticDefenses(i, 1) + mapFeatures.staticDefenses(i, 3);
    R = mapFeatures.staticDefenses(i, 3);
    y1 = sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
    y2 = -sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
    plot(x, y1, x, y2,'Color',"r");
end

%Plot mobile defenses once
for j = 1:height(mapFeatures.mobileDefenses)
    x = mapFeatures.mobileDefenses(j, 1) - mapFeatures.mobileDefenses(j, 3):1:mapFeatures.mobileDefenses(j, 1) + mapFeatures.mobileDefenses(j, 3);
    R = mapFeatures.mobileDefenses(j, 3);
    y1 = sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
    y2 = -sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
    plot(x, y1, x, y2,'Color',"b");
end

%% Start Monte-Carlo

% Define parameters for Monte Carlo analysis
maxIterations = 10;  % # of iterations for Monte Carlo
killVar = zeros(maxIterations, 1);  % Initialize
killXY = NaN(maxIterations, 2);  % Initialize

% Store success rate& standard deviation history
successRateHistory = zeros(maxIterations, 1);
stdSuccessRate = zeros(maxIterations, 1); 

% Initialize success rate plot
figure(2);
successPlot = plot(nan, nan, 'b-', 'LineWidth', 2); % Empty plot
xlabel('Number of Simulations');
ylabel('Defense Success Rate (%)');
title('Defense Success Rates Over Simulation');
grid on;
hold on;

% Initialize standard deviation plot
figure(3);
stdPlot = plot(nan, nan, 'r-', 'LineWidth', 2); % Empty plot
xlabel('Number of Simulations');
ylabel('Standard Deviation of Success Rate (%)');
title('Standard Deviation of Defense Success Rate Over Simulation');
grid on;

% Randomize randi seed for this session
rng('shuffle')

% Start Monte Carlo loop
for N = 1:maxIterations
%% Hybrid A*
% Generate UAS path using a Hybrid A* path planning algorithm. The while
% loop accounts for the rare event when the Hybrid A* function returns an
% empty path so it reruns the function

maxAttempts = 5;
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
% Select closest defense
if height(mapFeatures.mobileDefenses) > 0
selectedMobileDefense = mobileDefenseSelection(height(mapFeatures.mobileDefenses), mapFeatures.mobileDefenses, uasPosition);
mDInitialPosition = mapFeatures.mobileDefenses(selectedMobileDefense, 1:2);
mobileDefensePosition = mobileDefensePathing(uasPosition, mDInitialPosition, mobileDefenseSpeed, dT);
else
    mobileDefensePosition = [NaN, NaN];
    selectedMobileDefense = NaN;
end

%% Kill Chain logic
%returns new terminated flight tracks and positions of kill
%[updatedAdversaryPosition, killPoints] = killChain(uasPosition, mapFeatures);
[SDHits, MDHits] = killDetection(mapFeatures, uasPosition, mobileDefensePosition, selectedMobileDefense);
updatedAdversaryPosition = uasPosition;
[killVar(N), ~, killXY(N, :)] = killCheck(SDHits, MDHits, trackProb, killProb, mapFeatures, uasPosition);

%% Calculate and plot success rate

% Calculate defense success rate
defenseRate = sum(killVar(1:N)) / N * 100;
successRateHistory(N) = defenseRate;

% Calculate standard deviation based on all completed simulations
stdSuccessRate(N) = std(successRateHistory(1:N));

% Update success rate plot
set(successPlot, 'XData', 1:N, 'YData', successRateHistory(1:N));

% Update standard deviation plot
set(stdPlot, 'XData', 1:N, 'YData', stdSuccessRate(1:N));

% Refresh plots
drawnow;

%% Plot the rest of the simulated environment
% % Plot the map, map features, and UAS track

% Adjust plot settings
xlim([mapBounds(1), mapBounds(2)]);
ylim([mapBounds(3), mapBounds(4)]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title(sprintf('UAS Simulation After %d Iterations', N));
grid on;

% Delete prior mobile defense movement path
delete(findobj(gca, 'Tag', 'dynamic'));

% Plot mobile defense movement as a continuous path
plot(mobileDefensePosition(:, 1), mobileDefensePosition(:, 2), 'c-', 'LineWidth', 1.5, 'DisplayName', 'Mobile Defense Path', 'Tag', 'dynamic');
scatter(mobileDefensePosition(:, 1), mobileDefensePosition(:, 2), 10, 'c', 'filled', 'Tag', 'dynamic');

% Plot mobile defense kill marker
if killVar(N) == 1
    plot(killXY(N, 1), killXY(N, 2), 'gx', 'MarkerSize', 1, 'LineWidth', 1)
end

end % End of Monte-Carlo loop