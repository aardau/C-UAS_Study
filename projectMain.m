%% Main script for Counter-UAS simulation(s)
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
mobileTrackProb = 0.75 * staticTrackProb; % X% as effective
staticKillProb = 0.5;
mobileKillProb = 0.75 * staticKillProb; % X% as effective

% % Specify file name for defense placements
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
% map features like the base, obstactles, and defense starting positions
plotStaticMapFeatures(mapBounds, mapFeatures);

%% Start Monte-Carlo

% Define parameters for Monte Carlo analysis
maxIterations = 5;  % # of iterations for Monte Carlo
killVar = zeros(maxIterations, 1);  % Initialize
killXY = NaN(maxIterations, 2);  % Initialize

% Store success rate& standard deviation history
successRateHistory = zeros(maxIterations, 1);
stdSuccessRate = zeros(maxIterations, 1); 
ciHistory = zeros(maxIterations, 1);
lowerCIHistory = zeros(maxIterations, 1);
upperCIHistory = zeros(maxIterations, 1);

% Initialize success rate plot
figure(2);
successPlot = plot(nan, nan, 'o-', 'LineWidth', 1);
xlabel('Number of Simulations');
ylabel('Defense Success Rate (%)');
title('Cumulative Defense Success Rate');
ylim([0 100])
grid on;
hold on;

% Initialize standard deviation plot
figure(3);
stdPlot = plot(nan, nan, 'o-', 'LineWidth', 1);
xlabel('Number of Simulations');
ylabel('Standard Deviation of Success Rate (%)');
title('Standard Deviation of Defense Success Rate');
grid on;

% Initialize confidence interval plot
figure(4);
ciErrorBar = errorbar(nan, nan, nan, nan, 'o-', 'LineWidth', 1);
xlabel('Number of Simulations');
ylabel('Defense Success Rate (%)');
title('Clopper-Pearson Confidence Interval');
ylim([0 100])
grid on;

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
[SDHits, MDHits] = killDetection(mapFeatures, uasPosition, mobileDefensePosition, selectedMobileDefense);
[killVar(N), killTimeStep, killXY(N, :)] = killCheck(SDHits, MDHits, staticTrackProb, mobileTrackProb, staticKillProb, mobileKillProb, mapFeatures, uasPosition);

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


%% Calculate and plot success rate, standard deviation, & confidence interval

% Calculate defense success rate
defenseRate = sum(killVar(1:N)) / N * 100;
successRateHistory(N) = defenseRate;

% Calculate standard deviation based on all completed simulations
stdSuccessRate(N) = std(successRateHistory(1:N));

% Calculate Clopper-Pearson confidence interval using Stats toolbox
[phat, pci] = binofit(sum(killVar(1:N)), N, 0.05);  % 95% confidence interval

% Convert confidence interval percentages
ciHistory(N) = phat * 100;
lowerCIHistory(N) = pci(1) * 100;
upperCIHistory(N) = pci(2) * 100;

% Compute error bar lengths
error_lower = successRateHistory(1:N) - lowerCIHistory(1:N);
error_upper = upperCIHistory(1:N) - successRateHistory(1:N);

% Update success rate plot
set(successPlot, 'XData', 1:N, 'YData', successRateHistory(1:N));

% Update standard deviation plot
set(stdPlot, 'XData', 1:N, 'YData', stdSuccessRate(1:N));

% Update confidence interval plot
set(ciErrorBar, 'XData', 1:N, 'YData', successRateHistory(1:N), ...
    'LData', successRateHistory(1:N) - lowerCIHistory(1:N), ...
    'UData', upperCIHistory(1:N) - successRateHistory(1:N));

% Refresh plots
drawnow;

%% Plot the dynamic elements of the simulated environment

% Adjust plot settings
title(sprintf('UAS Simulation After %d Iterations', N));

% Delete prior mobile defense movement path
figure(1)
delete(findobj(gca, 'Tag', 'dynamic'));

if height(mapFeatures.mobileDefenses) > 0
    % Plot mobile defense movement (continuous path w/ position points)
    plot(mobileToPlot(:,1), mobileToPlot(:,2), 'c-', 'LineWidth', 1.5, 'Tag', 'dynamic');
    scatter(mobileToPlot(:,1), mobileToPlot(:,2), 10, 'c', 'filled', 'Tag', 'dynamic');
end

% Plot the UAS path (continuous path w/ position points)
plot(uasToPlot(:,1), uasToPlot(:,2), 'm-', 'LineWidth', 1.5, 'Tag', 'dynamic');
scatter(uasToPlot(:,1), uasToPlot(:,2), 10, 'm', 'filled', 'Tag', 'dynamic');

% Plot defense kill marker
if killVar(N) == 1
    % Only plot if there is a kill
    if ~isnan(killTimeStep)
        % Count the number of hits from mobile and static defenses at killTimeStep
        mobileHitCount = sum(MDHits(killTimeStep, :));
        staticHitCount = sum(SDHits(killTimeStep, :));
        
        % Choose the color based on which defense contributed more hits.
        if mobileHitCount >= staticHitCount
            % Blue for mobile defense kill marker
            plot(killXY(N, 1), killXY(N, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2)
        else
            % Red for static defense kill marker
            plot(killXY(N, 1), killXY(N, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
        end
    end
end

end % End of Monte-Carlo loop

% Clear paths from last iteration in figure 1
figure(1)
delete(findobj(gca, 'Tag', 'dynamic'));

% End of simulation information
fprintf('Simulation completed after %d Iterations\n', N);
fprintf('Final Defense Success Rate: %.2f%%\n', successRateHistory(end));
fprintf('Standard Deviation of Success Rate: %.2f%%\n', stdSuccessRate(end));
fprintf('95%% Confidence Interval: [%.2f%%, %.2f%%]\n', lowerCIHistory(end), upperCIHistory(end));