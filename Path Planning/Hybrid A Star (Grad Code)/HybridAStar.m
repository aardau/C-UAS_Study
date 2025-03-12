%%%Backtracking Hybrid A* Graph Search%%%

reached_goal = 0;

tic

num_actions = 3;

%frontier structure
frontier = struct();
frontier.cost = 0;
frontier.starcost = 0;
frontier.load = 0;
frontier.idx = start_index;
frontier.node = start;
frontier.parent = -1;
frontier.action = -1;
    
%visited structure
visited = zeros(numgraph, 1);
parent = zeros(numgraph, 1);
child = zeros(numgraph, 3);
parent_act = zeros(numgraph, 1);
node = zeros(numgraph, N);
cost = zeros(numgraph, 1);
load = zeros(numgraph, 1);

%child vals => -1 in frontier, 0 > in visited, 0 doesnt exist

while ~isempty(frontier.cost)
    %assign the current node
    current = struct();
    current.cost = frontier.cost(1);
    current.starcost = frontier.starcost(1);
    current.load = frontier.load(1);
    current.idx = frontier.idx(1);
    current.node = frontier.node(1, :);
    current.parent = frontier.parent(1);
    current.action = frontier.action(1);
    
    frontier.cost = frontier.cost(2:end);
    frontier.starcost = frontier.starcost(2:end);
    frontier.load = frontier.load(2:end);
    frontier.idx = frontier.idx(2:end);
    frontier.node = frontier.node(2:end, :);
    frontier.parent = frontier.parent(2:end);
    frontier.action = frontier.action(2:end);
    
    %pop the node if it has been visited
    if visited(current.idx) == 1
        continue;
    end
    
    if current.load >= loadfunc.thres{1}
        if plot_steps == 2
            hold on;
            b_idx = current.parent;
            b_aci = current.action;
            b_node = current.node;
            while true
                bcurrent_idx = b_idx(end);
                bparent_idx = parent(bcurrent_idx);
                if bcurrent_idx ~= start_index
                    b_node = [b_node; node(bcurrent_idx, :)];
                    b_idx = [b_idx; bparent_idx];
                    b_aci = [b_aci; parent_act(bcurrent_idx)];
                else
                    break;
                end
            end
            
            b_node = flip(b_node);
            b_aci = flip(b_aci);
            for i = 1:length(b_aci)-1
                action = drawaction(b_node(i,:), L, R, b_aci(i+1), cost_discretization);
                blockpath = plot(action(:,1), action(:,2), 'LineWidth', 1, 'Color', 'b');
            end
        end
        continue;
    end
    
    %add the node to the visited set
    visited(current.idx) = 1;
    parent(current.idx) = current.parent;
    parent_act(current.idx) = current.action;
    node(current.idx, :) = current.node;
    cost(current.idx) = current.cost;
    load(current.idx) = current.load;
    
    if current.idx ~= start_index
        child(current.parent, current.action) = current.idx;
    end
    
    if current.idx == goal_index
        reached_goal = 1;
        break
    end
    
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
        new_cost = integrate_cost(candpath, speed, costfunc);
        new_load = integrate_cost(candpath, speed, loadfunc.eval{1});
        
        newnode.cost = current.cost + new_cost;
        newnode.starcost = newnode.cost + norm(newnode.node(1:2) - goal(1:2));
        newnode.load = current.load + new_load;
        newnode.action = aci;
        
        %push the node into the frontier
        relax = true;
        for j = 1:length(frontier.cost)
            if (frontier.idx(j) == newnode.idx)
                if frontier.starcost(j) >= newnode.starcost
                    frontier.node(j, :) = [];
                    frontier.parent(j) = [];
                    frontier.idx(j) = [];
                    frontier.cost(j) = [];
                    frontier.starcost(j) = [];
                    frontier.load(j) = [];
                    frontier.action(j) = [];
                    break;
                elseif frontier.starcost(j) < newnode.starcost
                    relax = false;
                end
            end
        end
        
        if relax
            frontier_length = length(frontier.cost);
            entrylocation = find(frontier.starcost > newnode.starcost, 1);
            if isempty(entrylocation)
                entrylocation = frontier_length + 1;
            end
        
            frontier.cost = [frontier.cost(1:entrylocation-1); newnode.cost; frontier.cost(entrylocation:end)];
            frontier.starcost = [frontier.starcost(1:entrylocation-1); newnode.starcost; frontier.starcost(entrylocation:end)];
            frontier.load = [frontier.load(1:entrylocation-1); newnode.load; frontier.load(entrylocation:end)];
            frontier.node = [frontier.node(1:entrylocation-1, :); newnode.node; frontier.node(entrylocation:end, :)];
            frontier.idx = [frontier.idx(1:entrylocation-1); newnode.idx; frontier.idx(entrylocation:end)];
            frontier.parent = [frontier.parent(1:entrylocation-1); newnode.parent; frontier.parent(entrylocation:end)];
            frontier.action = [frontier.action(1:entrylocation-1); newnode.action; frontier.action(entrylocation:end)];
        end
    end
end
runtime = toc;

if (plot_steps > 0) && (reached_goal == true)
    %reconstruct the path
    path = current.node;
    path_aci = current.action;
    current_idx = current.idx;

    while current_idx ~= -1
        current_node = node(current_idx, :);
        current_aci = parent_act(current_idx);
        current_idx = parent(current_idx);
        path = [path; current_node];
        path_aci = [path_aci; current_aci];
    end
    
    figure(1);
    hold on;
    path = flip(path);
    path_aci = flip(path_aci);
    for i = 1:length(path_aci)-2
        action = drawaction(path(i,:), L, R, path_aci(i+1), cost_discretization);
        final = plot(action(:,1), action(:,2), 'LineWidth', 3, 'Color', 'magenta');
    end
end

fprintf("Terminal Cost: %f\n", current.cost);
fprintf("Terminal Load: %f\n", current.load);
fprintf("Runtime: %f s\n", runtime);

if (reached_goal == true) && (plot_steps == 2)
    legend([blockpath, final], 'Loading Limit Reached', 'HA* Solution');
elseif plot_steps == 2
    legend(blockpath, 'Loading Limit Reached');
end