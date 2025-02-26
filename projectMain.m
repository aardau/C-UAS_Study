%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

for N = 1:1
%% Define parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Generation parameters for setupMapFile (current)
% effector generation parameters
rangeMin = 500;
rangeMax = 1000;
mobileDefenseSpeed = 10;

% create limits matrix
limits = [rangeMin, rangeMax;];

% probabilities
trackProb = 0.1;
killProb = 0.5;

% specify file name for input
%fn = "effector_inputs.xlsx"; % all static, lots of randomization
fn = "effector_inputs_single_static.xlsx"; % entirely specified starting condition, 3S 2M
%fn = "effector_inputs_RAND.xlsx"; % entirely randomized setup
effectorData = readmatrix(fn);

% Generation parameters for setupMap (old)
% Defenses generation parameters
rangeStatic = 200; % [min,max] of detection range in arbitrary length units
rangeMobile = 100; % [min,max] of detection range in arbitrary length units
defenseRanges = [rangeStatic, rangeMobile]; % For funcion inputs

numMaxDefenses = height(effectorData); % Maximum number of defenses that spawn
% currently unused variables
%numStaticDefenses = randi([0,numMaxDefenses]); % Generate a random number of static defenses, including zero
%numMobileDefenses = numMaxDefenses - numStaticDefenses; % Generate mobile defenses with remaining # of slots

% UAS parameters
velUAS = 20; % Velocity (units/s)
maxThetaUAS = 15; % Maximum turn angle (deg)
dT = 1; % Time step (s) (Don't change to 0.1, it takes too long to compute HA*)

% Mobile Defense parameters
velMD = 5; % Velocity, (units/s)
maxThetaMD = 360; % Maximum turn angle (deg), setting to 360deg makes the vehicle behave closer to A* path finding

%% Setup map features
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

%% Run Hybrid A*
% Generate UAS path using a Hybrid A* path planning algorithm
uasPath = hybridAStarFunc(mapBounds, mapFeatures, velUAS, maxThetaUAS, dT);

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

%% Plot
% Plot the map, map features, and UAS track
plotMap(mapFeatures, mapBounds, updatedAdversaryPosition, mobileDefensePosition);

% TEST CODE FOR PLOTTING MOBILE DEFENSE PATH -- WILL BE ADDED TO PLOT MAP
% AT LATER DATE -- SOMETIMES GIVES ERROR BC NO MOBILE DEFENSES WERE
% GENERATED
hold on
x = mobileDefensePosition(:, 1); y = mobileDefensePosition(:, 2);
plot(x, y, 'cx')
if killVar(N) == 1
plot(killXY(N, 1), killXY(N, 2), 'gx', 'MarkerSize', 30, 'LineWidth', 8)
end

end % End of Monte-Carlo loop

%% Calculations after sims
% calculate success rate
defenseRate = sum(killVar)/N * 100;
fprintf("\nThe UAS was stopped in %.0f percent of runs.\n", defenseRate)
