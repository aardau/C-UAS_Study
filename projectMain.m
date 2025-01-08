%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Map generation parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually
xmin = -2500; xmax = 2500;
ymin = -2500; ymax = 2500;
rangemin = 500; rangemax = 500;
numEffectors = randi(3);

% UAS generation parameters for simulateUAS
uasParameters = [0, 0, 70, 22, 40];
fixedPoint = [0, 2500, 100];
pidTune = [0.5, 0.1, 0.05];
simParameters = [0.1, 500];

%% Setup the map
% Gather the map features generated in the setupMap function and place into a structure array
mapFeatures = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors);

%% Simulate the UAS
[xposUAS, yposUAS, totalDistanceToFP] = simulateUAS(uasParameters, fixedPoint, pidTune, simParameters);

%% Plot the map, map features, and UAS
plotMap(mapFeatures, xmin, xmax, ymin, ymax, xposUAS, yposUAS, fixedPoint);