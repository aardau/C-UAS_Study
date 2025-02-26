function trajectory = HAstarTrajectory(mapData, startPose, goalPose, maxVel, maxTurnAngleDeg, dt)
    %#codegen

    % Create a binary occupancy map
    map = binaryOccupancyMap(mapData);

    % Create a state space object
    stateSpace = stateSpaceSE2;
    stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

    % Construct a state validator object using the state space and map
    validator = validatorOccupancyMap(stateSpace, Map=map);
    validator.ValidationDistance = 0.01; % Ensures smoother path validation

    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);

    % Convert max turn angle to minimum turning radius
    L = 2; % Effective wheelbase (tunable)
    minTurnRadius = L / tand(maxTurnAngleDeg);
    planner.MinTurningRadius = minTurnRadius;

    planner.MotionPrimitiveLength = 2; % Controls movement per step
    planner.ReverseCost = 1e6; % UAV cannot move in reverse

    % Compute the initial path
    pathObj = plan(planner, startPose, goalPose);
    path = pathObj.States; % Extract waypoints

    % === Time-Based Simulation ===
    numSteps = 1000; % Preallocate large array (adjust as needed)
    trajectory = zeros(numSteps, 3); % [x, y, theta]
    trajectory(1, :) = startPose;

    currentPose = startPose;
    idx = 2;

    % Convert max turn angle to radians
    maxTurnAngleRad = deg2rad(maxTurnAngleDeg);

    % Define a tolerance for stopping adjustments
    headingTolerance = deg2rad(5);  % Allow small deviations without correction

    % Iterate over each waypoint
    for i = 2:size(path, 1)
        nextPose = path(i, :);
        distance = norm(nextPose(1:2) - currentPose(1:2));

        % Move towards next waypoint using kinematic constraints
        while distance > 0.1 % Threshold to consider a waypoint reached
            if idx > numSteps
                trajectory = trajectory(1:idx-1, :); % Trim excess zeros
                return;
            end

            % Compute desired direction
            desiredTheta = atan2(nextPose(2) - currentPose(2), nextPose(1) - currentPose(1));
            headingError = atan2(sin(desiredTheta - currentPose(3)), cos(desiredTheta - currentPose(3)));

            % Apply proportional heading correction with max turn angle limit
            if abs(headingError) > headingTolerance
                deltaTheta = sign(headingError) * min(abs(headingError), maxTurnAngleRad);
                currentPose(3) = currentPose(3) + deltaTheta;
            end

            % Apply bicycle kinematics model for position update
            dx = maxVel * cos(currentPose(3)) * dt;
            dy = maxVel * sin(currentPose(3)) * dt;

            % Ensure movement towards the waypoint
            stepSize = norm([dx, dy]);
            if stepSize > distance
                dx = (dx / stepSize) * distance;
                dy = (dy / stepSize) * distance;
            end

            % Update UAV state
            currentPose(1) = currentPose(1) + dx;
            currentPose(2) = currentPose(2) + dy;

            % Store trajectory
            trajectory(idx, :) = currentPose;
            idx = idx + 1;

            % Recalculate remaining distance to waypoint
            distance = norm(nextPose(1:2) - currentPose(1:2));
        end
    end

    % Trim excess zeros
    trajectory = trajectory(1:idx-1, :);
end
