%% Main script for Counter-UAS simulations
clear; clc; close all;

%% Define parameters

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Effector generation parameters
rangeMin = 500;
rangeMax = 500;
mobileDefenseSpeed = 5;

% Create limits matrix
limits = [rangeMin, rangeMax;];

% Kill-chain probabilities
staticTrackProb = 0.1;
mobileTrackProb = 0.75 * staticTrackProb; % 75% as effective as static
staticKillProb = 0.5;
mobileKillProb = 0.75 * staticKillProb; % 75% as effective as static

% % Specify file name for defense placements and extract the data
fn = "benchmark_mobile_3.xlsx"; 
effectorData = readmatrix(fn);

% UAS parameters
velUAS = 20; % Velocity (units/s)
minTurnRad = 8; % Minimum turn radius (units)
dT = 1; % Time step (s) (don't change)

% Randomize randi seed for this session
rng('shuffle')

%% Generate map features 
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

%% Plot run-independent map features once
% Initializes the simulated environment figure and plots run-independent
% map features like the base, obstacles, and defense starting positions
plotStaticMapFeatures(mapBounds, mapFeatures);

%% Start Monte-Carlo

% Define the maximum number of iterations for the Monte Carlo analysis
maxIterations = 5;

% Initializes Monte Carlo analysis data structure and their associated plots
[MCData, plotHandles] = plotMonteCarloData(maxIterations);

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

% Extract the (x,y) coordinates from uasPath
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
[SDHits, MDHits] = killDetection(mapFeatures, uasPosition, mobileDefensePosition, selectedMobileDefense);
[MCData.killVar(N), killTimeStep, MCData.killXY(N, :)] = killCheck(SDHits, MDHits, staticTrackProb, mobileTrackProb, staticKillProb, mobileKillProb, mapFeatures, uasPosition);

% If kill, plot the truncated path
if ~isnan(killTimeStep)
    uasToPlot = uasPosition(1:killTimeStep, :);
    if height(mapFeatures.mobileDefenses > 0)
        mobileToPlot = mobileDefensePosition(1:killTimeStep, :);
    end
else
    % If no kill, plot the entire path
    uasToPlot = uasPosition;
    if height(mapFeatures.mobileDefenses > 0)
        mobileToPlot = mobileDefensePosition;
    end
end

%% Calculate Monte Carlo data for each iteration
% Calculates the defense success rate, standard deviation, and 95%
% Clopper-Pearson confidence interval for each iteration
MCData = calculateMonteCarloData(MCData, N);

%% Plots dynamic map features
% Dynamically updates the environment map (figure 1) and the three data
% maps (figures 2-4)
plotDynamicMapFeatures(N, MCData, plotHandles, uasToPlot, mobileToPlot, killTimeStep, MDHits, SDHits, mapFeatures);

end % End of Monte-Carlo loop

% Clear paths from last iteration in figure 1
figure(1)
delete(findobj(gca, 'Tag', 'dynamic'));

% End of simulation information
fprintf('Simulation completed after %d Iterations\n', maxIterations);
fprintf('Final Defense Success Rate: %.2f%%\n', MCData.successRateHistory(end));
fprintf('Standard Deviation of Success Rate: %.2f%%\n', MCData.stdSuccessRate(end));
fprintf('95%% Confidence Interval: [%.2f%%, %.2f%%]\n', MCData.lowerCIHistory(end), MCData.upperCIHistory(end));