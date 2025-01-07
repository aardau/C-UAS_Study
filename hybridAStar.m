function path = hybridAStar(start, goal, map, params)
    % Inputs:
    % -start: [x, y, θ] initial state
    % -goal: [x, y, θ] target state
    % -map: Occupancy grid or obstacle map
    % -params: Parameters for the Hybrid A* algorithm (e.g., resolution, cost weights)
    
    % Outputs:
    % -path: Planned path as a series of states [x, y, θ]

    % Initialize open and closed lists
    openList = PriorityQueue(); % Custom priority queue for A*
    closedList = [];
    
    % Add the start node
    startNode = createNode(start, 0, heuristic(start, goal));
    openList.insert(startNode);

    while ~openList.isEmpty()
        % Get the node with the lowest cost
        currentNode = openList.pop();
        
        % Check if goal is reached
        if isGoal(currentNode.state, goal, params)
            path = reconstructPath(currentNode);
            return;
        end
        
        % Generate motion primitives and expand nodes
        successors = generateSuccessors(currentNode, map, params);
        for successor = successors
            if ~isInClosedList(successor, closedList)
                openList.insert(successor);
            end
        end
        
        % Add current node to closed list
        closedList = [closedList; currentNode];
    end
    
    error('Path not found');
end