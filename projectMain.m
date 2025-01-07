%% Main script for Counter-UAS simulation(s)
clear; clc; close all;

%% Define parameters
% Map generation parameters
% Allow the projectMainApp (user GUI) to change these numbers eventually
xmin = -2500; xmax = 2500;
ymin = -2500; ymax = 2500;
rangemin = 100; rangemax = 100;
numEffectors = randi(3);

% UAS generation parameters
uasParameters = [0, 0, 70, 22, 40];
fixedPoint = [1750, 1750, 50];
pidTune = [0.5, 0.1, 0.05];
simParameters = [0.1, 500];

%% Setup and plot map
% Gather the map data generated in tbe setupMap function and place into a structure array
mapData = setupMap(xmin, xmax, ymin, ymax, rangemin, rangemax, numEffectors);
% Plot the map and map data
plotMap(mapData, xmin, xmax, ymin, ymax);

%% Simulate UAS
[x, y, totalDistanceToFP] = simulateUAS(uasParameters, fixedPoint, pidTune, simParameters);

%% Plot UAS simulation
plotSimulation(x, y, fixedPoint);