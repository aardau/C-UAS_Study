%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Generation parameters for setupMapFile (current)
% effector generation parameters
rangeMin = 100;
rangeMax = 200;
mobileDefenseSpeed = 10;

% create limits matrix
limits = [rangeMin, rangeMax;];

% specify file name for input
fn = "effector_inputs_RAND.xlsx";
effectorData = readmatrix(fn);

% Generation parameters for setupMap (old)
% Defenses generation parameters
rangeStatic = [500, 500]; % [min,max] of detection range in arbitrary length units
rangeMobile = [500, 500]; % [min,max] of detection range in arbitrary length units
defenseRanges = [rangeStatic, rangeMobile]; % For saving space when writing function inputs

numMaxDefenses = 3; % Maximum number of defenses that spawn
numStaticDefenses = randi([0,numMaxDefenses]); % Generate a random number of static defenses, including zero
numMobileDefenses = numMaxDefenses - numStaticDefenses; % Generate mobile defenses with remaining # of slots
numDefenses = [numStaticDefenses, numMobileDefenses]; % For saving space when writing function inputs

% UAS generation parameters
vel = 20; % Velocity (units/s)
maxTheta = 10; % Maximum turn angle (deg)
dT = 1; % Time step (s)
iterUAS = 1000; % # of iterations

%% Setup map features
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

%% Simulate UAS
% Simulate the UAS using bicycle kinematics and return [x,y] positions of
% UAS track
adversaryPosition = simulateUAS(mapBounds, vel, maxTheta, dT, iterUAS);

%% Mobile Defense Movement
mDInitialPosition = mapFeatures.mobileDefenses(1, 1:2);
mobileDefensePosition = mobileDefensePathing(adversaryPosition, mDInitialPosition, mobileDefenseSpeed, dT);

%% Kill Chain
%returns new terminated flight tracks and positions of kill
[updatedAdversaryPosition, killPoints] = killChain(adversaryPosition,mapFeatures);

%% Plot
% Plot the map, map features, and UAS track
plotMap(mapFeatures, mapBounds, updatedAdversaryPosition, mobileDefensePosition);

% TEST CODE FOR PLOTTING MOBILE DEFENSE PATH -- WILL BE ADDED TO PLOT MAP
% AT LATER DATE
hold on
x = mobileDefensePosition(1, :); y = mobileDefensePosition(2, :);
plot(x, y, 'cx')
