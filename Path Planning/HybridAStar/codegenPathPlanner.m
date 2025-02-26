function path = codegenPathPlanner(mapData,startPose,goalPose)
    %#codegen
       
    % Create a binary occupancy map
    map = binaryOccupancyMap(mapData);

    % Create a state space object
    stateSpace = stateSpaceSE2;

    % Update state space bounds to be the same as map limits.
    stateSpace.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];

    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace,Map=map);
    
    % Set the validation distance for the validator
    validator.ValidationDistance = 0.01;
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);
    
    % Define kinematic constraints
    planner.MinTurningRadius = 8; % Minimum turn radius in meters
    planner.MotionPrimitiveLength = 2; % Step size for motion primitives
    planner.ForwardCost = 1; % Cost of moving forward
    planner.ReverseCost = 1e6; % UAS cannot reverse
    planner.DirectionSwitchingCost = 5; % Penalizes direction changes
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner,startPose,goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;

end

