%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Map generation parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually
xmin = -2500; xmax = 2500;
ymin = -2500; ymax = 2500;
rangemin = 500; rangemax = 500;
numEffectors = randi(3);

% UAS generation parameters
uasParameters = [0, 0, 70, 22, 40];
fixedPoint = [0, 2500, 100];
pidTune = [0.5, 0.1, 0.05];
simParameters = [0.1, 500];

%% Setup and plot map
% Gather the map features generated in tbe setupMap function and place into a structure array
mapData = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors);
% Plot the map and map features
plotMap(mapData, xmin, xmax, ymin, ymax);

%% Setup the UAS
[x, y, totalDistanceToFP] = simulateUAS(uasParameters, fixedPoint, pidTune, simParameters);

%% Plot the UAS on the map
plotSimulation(x, y, fixedPoint);