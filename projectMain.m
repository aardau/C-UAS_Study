%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% Defenses generation parameters
rangeStatic = [500, 500]; % [min,max] of detection range in arbitrary length units
rangeMobile = [500, 500]; % [min,max] of detection range in arbitrary length units
defenseRanges = [rangeStatic, rangeMobile]; % For saving space when writing function inputs

numStaticDefenses = randi([0,3]); % Generate a random number of static defenses, including zero
numMobileDefenses = 3 - numStaticDefenses; % Generate mobile defenses with remaining # of slots
numDefenses = [numStaticDefenses, numMobileDefenses]; % For saving space when writing function inputs

% UAS generation parameters
vel = 50; % Velocity (units/s)
maxTheta = 10; % Maximum turn angle (deg)
dT = 1; % Time step (s)
iterUAS = 1000; % # of iterations

%% Setup map features
% Generate the various map features and place into a structure array
mapFeatures = setupMap(mapBounds, defenseRanges, numDefenses);

%% Simulate UAS
% Simulate the UAS using bicycle kinematics and return [x,y] positions of
% UAS track
[xposUAS, yposUAS] = simulateUAS(mapBounds, vel, maxTheta, dT, iterUAS);

%% Plot
% Plot the map, map features, and UAS track
plotMap(mapFeatures, mapBounds, xposUAS, yposUAS);
