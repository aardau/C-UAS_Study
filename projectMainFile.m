%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually

% Map generation parameters
mapBounds = [-2500, 2500, -2500, 2500]; % [xmin, xmax, ymin, ymax] in arbitrary length units

% effector generation parameters
rangeMin = 100;
rangeMax = 200;

% create limits matrix
limits = [rangeMin, rangeMax;];

% specify file name for input
fn = "effector_inputs_RAND.xlsx";
effectorData = readmatrix(fn);

% UAS generation parameters
vel = 50; % Velocity (units/s)
maxTheta = 10; % Maximum turn angle (deg)
dT = 1; % Time step (s)
iterUAS = 1000; % # of iterations

%% Setup map features
% Generate the various map features and place into a structure array
mapFeatures = setupMapFile(mapBounds, effectorData, limits);

%% Simulate UAS
% Simulate the UAS using bicycle kinematics and return [x,y] positions of
% UAS track
[xposUAS, yposUAS] = simulateUAS(mapBounds, vel, maxTheta, dT, iterUAS);

%% Plot
% Plot the map, map features, and UAS track
plotMap(mapFeatures, mapBounds, xposUAS, yposUAS);
