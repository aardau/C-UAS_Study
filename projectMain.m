%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Map generation parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually
xmin = -2500; xmax = 2500;
ymin = -2500; ymax = 2500;
bounds = [xmin, xmax, ymin, ymax]; % For ease of use in calling functions
rangemin = 500; rangemax = 500;
numEffectors = randi(3);

% UAS generation parameters for SimulateUAS
vel = 50; % Velocity (m/s)
maxTheta = 5; % Maximum turn angle (deg)
dT = 1; % Time step (s)
iterUAS = 1000; % # of iterations

%% Setup the map
% Gather the map features generated in the setupMap function and place into a structure array
mapFeatures = setupMap(bounds, rangemin, rangemax, numEffectors);

%% Simulate the UAS
[xposUAS, yposUAS] = simulateUAS(bounds, vel, maxTheta, dT, iterUAS);

%% Plot the map, map features, and UAS
plotMap(mapFeatures, bounds, xposUAS, yposUAS);
