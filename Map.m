% add description
close all; clear; clc

% define region size
% working in SI units (m/kg/etc)
xmin = -2500; xmax = 2500; ymin = xmin; ymax = xmax;
rangemin = 100; rangemax = 1500;

% take in effector data file
effector_parameters = readmatrix("effector_input.xlsx");

% randomly generate unknown parameters
hold on
for i=1:height(effector_parameters)
    % randomly generate non-set parameters
    if isnan(effector_parameters(i, 1))
        effector_parameters(i, 1) = randi([xmin, xmax]);
    end
    if isnan(effector_parameters(i, 2))
        effector_parameters(i, 2) = randi([ymin, ymax]);
    end
    if isnan(effector_parameters(i, 3))
        effector_parameters(i, 3) = randi([rangemin, rangemax]);
    end
    
    % create range bounds
    x = effector_parameters(i, 1)-effector_parameters(i, 3):1:effector_parameters(i, 1)+effector_parameters(i, 3);
    R = effector_parameters(i, 3);
    y1 = sqrt(R^2-(x-effector_parameters(i, 1)).^2 ) + effector_parameters(i, 2);
    y2 = -sqrt(R^2-(x-effector_parameters(i, 1)).^2 ) + effector_parameters(i, 2);
    plot(x, y1, x, y2)
end

% create center 'base'
basex = [-100, 100, 100, -100]; basey = [-100, -100, 100, 100];
pgon = polyshape(basex, basey);
plot(pgon);
xlabel("X Position (m)"); ylabel("Y Position (m)");


xlim([xmin; xmax]); ylim([ymin, ymax]);
