% This function generates the path of the UAS given initial defined parameters. 
% The UAS spawns along a random edge and paths to the origin.

% Inputs:
% bounds: Maximum and minimum map bounds measured from zero in the form 
%         [xmin, xmax, ymin, ymax]
% velocity: Constant forward velocity of the UAS
% maxTheta: Maximum turn angle in degrees
% dT: Time step for the simulation
% iterations: Maximum number of iterations used in loop to create UAS path

% Outputs:
% [x, y]: Array of UAS positions over time

function adversaryPosition = simulateUAS(bounds, velocity, maxTheta, dT, iterUAS)
    
    % Extract map bounds for calculations
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);

    % Choose a random edge of the map and spawn the UAS somewhere along it
    % The initial heading is set to be perpendicular to the spawn edge
    edge = randi(4);
    switch edge
        case 1 % Top edge
            x(1) = randi([xmin, xmax]);
            y(1) = ymax;
            theta = -pi/2;
        case 2 % Bottom edge
            x(1) = randi([xmin, xmax]);
            y(1) = ymin;
            theta = pi/2;
        case 3 % Left edge
            x(1) = xmin;
            y(1) = randi([ymin, ymax]);
            theta = 0;
        case 4 % Right edge
            x(1) = xmax;
            y(1) = randi([ymin, ymax]);
            theta = pi;
        otherwise % Prints warning if no case is satisfied
            warning('Error in UAS spawning. Check simulateUAS function.')
    end

    % Simulation loop
    for i = 1:iterUAS
        % Calculate angle to origin
        thetaToTarget = atan2(0 - y(i), 0 - x(i)); %target is origin, (0,0)
        
        % Calculate the angle difference between the start and target
        %angleDiff = wrapToPi(thetaToTarget - theta); (old method, needs toolbox)
        angleDiff = mod((thetaToTarget - theta) + pi, 2*pi) - pi;

        % Limit turn angle to maxTurnAngle
        turnAngle = max(min(angleDiff, deg2rad(maxTheta)), -deg2rad(maxTheta));

        % Update heading
        theta = theta + turnAngle;

        % Update position using bicycle kinematics
        x(i+1) = x(i) + velocity * cos(theta) * dT;
        y(i+1) = y(i) + velocity * sin(theta) * dT;

        % Check if UAS reached the origin (within a small radius)
        if sqrt(x(i+1)^2 + y(i+1)^2) < velocity * dT
            x = x(1:i+1);
            y = y(1:i+1);
            return; % Exit simulation
        end
    end
    adversaryPosition = [x; y];
end


