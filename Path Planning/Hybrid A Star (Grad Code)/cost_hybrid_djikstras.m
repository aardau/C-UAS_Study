function [cost] = cost_hybrid_djikstras(start, start_index, grid_x, grid_y, grid_th, obstacles, cost_func, speed, L, R, N, cost_discretization, numgraph)
    
    num_actions = 3;

    %frontier structure
    frontier = struct();
    frontier.cost = 0;
    frontier.idx = start_index;
    frontier.node = start;
    frontier.parent = -1;
    frontier.action = -1;
    
    %visited structure
    visited = zeros(numgraph, 1);
    parent = zeros(numgraph, 1);
    parent_act = zeros(numgraph, 1);
    cost = zeros(numgraph, 1);
    node = zeros(numgraph, N);
    
    while ~isempty(frontier.cost)
        %assign the current node
        current = struct();
        current.cost = frontier.cost(1);
        current.idx = frontier.idx(1);
        current.node = frontier.node(1, :);
        current.parent = frontier.parent(1);
        current.action = frontier.action(1);
        
        %pop the current node from the frontier
        frontier.cost = frontier.cost(2:end);
        frontier.idx = frontier.idx(2:end);
        frontier.node = frontier.node(2:end, :);
        frontier.parent = frontier.parent(2:end);
        frontier.action = frontier.action(2:end);
        
        if visited(current.idx) == 1
            continue;
        end
        
        %add to the visited set
        visited(current.idx) = 1;
        parent(current.idx) = current.parent;
        node(current.idx, :) = current.node;
        cost(current.idx) = current.cost;
        parent_act(current.idx) = current.action;
        
        actions = get_actions_kinem(current.node, L, R, grid_x, grid_y, grid_th);
        for aci = 1:num_actions
            
            newnode = struct();
            newnode.node = actions.newlocation(aci, :);
            newnode.idx = actions.newindexc(aci);
            newnode.parent = current.idx;

            node_admissibility = check_admissibility(obstacles, newnode, current, R, aci);
             
            %check if the node is valid
            if ~node_admissibility
                continue;
            end
            if visited(newnode.idx) > 0 
                continue;
            end
            
            %compute the node cost using the cost function
            candpath = drawaction(current.node, L, R, aci, cost_discretization);
            new_cost = integrate_cost(candpath, speed, cost_func);
            
            newnode.cost = current.cost + new_cost;
            newnode.action = aci;
            
            %push the node into the frontier
            frontier_length = numel(frontier.cost);
            entrylocation = find(frontier.cost > newnode.cost, 1);
            if isempty(entrylocation)
                entrylocation = frontier_length + 1;
            end
            
            frontier.cost = [frontier.cost(1:entrylocation-1); newnode.cost; frontier.cost(entrylocation:end)];
            frontier.node = [frontier.node(1:entrylocation-1, :); newnode.node; frontier.node(entrylocation:end, :)];
            frontier.idx = [frontier.idx(1:entrylocation-1); newnode.idx; frontier.idx(entrylocation:end)];
            frontier.parent = [frontier.parent(1:entrylocation-1); newnode.parent; frontier.parent(entrylocation:end)];
            frontier.action = [frontier.action(1:entrylocation-1); newnode.action; frontier.action(entrylocation:end)];
        end
        
    end
end

