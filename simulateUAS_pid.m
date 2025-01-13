% This function xxxx

% Inputs
% uasParameters:
% fixedPoint:
% pidTune:
% simParameters:

% Outputs:
% x: x-position of the UAS
% y: y-position of the UAS
% totalDistancetoFP: Total distance to the fixed point


function [x, y, totalDistanceToFP] = simulateUAS_pid(uasParameters, fixedPoint, pidTune, simParameters)
    % Follow the path using PID parameters

    % Extract simulation parameters
    dT = simParameters(1);
    iterations = simParameters(2);

    % Pre-allocate variables
    x = zeros(1, iterations);
    y = zeros(1, iterations);
    theta = zeros(1, iterations);
    delta = zeros(1, iterations);
    v = zeros(1, iterations);
    phi = zeros(1, iterations);
    xDistanceToFP = zeros(1, iterations);
    yDistanceToFP = zeros(1, iterations);
    alphaAngleToFP = zeros(1, iterations);
    totalDistanceToFP = zeros(1, iterations);

    % Initialize UAS parameters
    x(1) = uasParameters(1);
    y(1) = uasParameters(2);
    theta(1) = uasParameters(3);
    v(:) = uasParameters(4);
    turnRadius = uasParameters(5);

    % PID parameters
    Kp = pidTune(1);
    Ki = pidTune(2);
    Kd = pidTune(3);
    error_prior = 0;
    integral_prior = 0;

    % Simulation loop
    for i = 1:iterations
        % Distance and angle to fixed point
        xDistanceToFP(i) = fixedPoint(1) - x(i);
        yDistanceToFP(i) = fixedPoint(2) - y(i);
        alphaAngleToFP(i) = atan2d(yDistanceToFP(i), xDistanceToFP(i));
        totalDistanceToFP(i) = sqrt(xDistanceToFP(i)^2 + yDistanceToFP(i)^2);
        
        % Stop if within end radius
        if totalDistanceToFP(i) < fixedPoint(3)
            x(i+1:end) = [];
            y(i+1:end) = [];
            break;
        end

        % PID control
        angleDif = alphaAngleToFP(i) - theta(i);
        integral = integral_prior + angleDif * dT;
        derivative = (angleDif - error_prior) / dT;
        phi(i) = Kp * angleDif + Ki * integral + Kd * derivative;

        % Update errors
        error_prior = angleDif;
        integral_prior = integral;

        % Equations of motion
        xDot = v(i) * cosd(delta(i) + theta(i));
        yDot = v(i) * sind(delta(i) + theta(i));
        thetaDot = v(i) * sind(delta(i));
        deltaDot = phi(i);

        x(i+1) = x(i) + xDot * dT;
        y(i+1) = y(i) + yDot * dT;
        theta(i+1) = theta(i) + thetaDot * dT;
        delta(i+1) = delta(i) + deltaDot * dT;

        % Enforce turn radius limits
        delta(i+1) = min(max(delta(i+1), -turnRadius), turnRadius);
    end
end