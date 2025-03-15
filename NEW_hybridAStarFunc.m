function [path] = NEW_hybridAStarFunc(mapBounds,mapFeatures, speed,R,DT)

% Extract map bounds for calculations
xmin = mapBounds(1); xmax = mapBounds(2);
ymin = mapBounds(3); ymax = mapBounds(4);

% Initialize start and end goal
start = [0,0,0];
goal = [0,0,0]; 

% Choose a random edge of the map to spawn the UAS
edge = randi(4);
switch edge
    case 1 % Top edge
        start(1) = randi([xmin, xmax]);
        start(2) = ymax;
        start(3) = 3*pi/2;
        goal(3) = start(3); % Keep the same heading as the start
    case 2 % Bottom edge
        start(1) = randi([xmin, xmax]);
        start(2) = ymin;
        start(3) = pi/2;
        goal(3) = start(3); % Keep the same heading as the start
    case 3 % Left edge
        start(1) = xmin;
        start(2) = randi([ymin, ymax]);
        start(3) = 0;
        goal(3) = start(3); % Keep the same heading as the start
    case 4 % Right edge
        start(1) = xmax;
        start(2) = randi([ymin, ymax]);
        start(3) = pi;
        goal(3) = start(3); % Keep the same heading as the start
    otherwise % Prints warning if no case is satisfied
        warning('Error in UAS spawning from switch cases')
end

% % This is optional code to change the goal from the origin to a random
% % point along the base perimeter
% % Extract base boundaries for goal determination
% baseXmin = min(mapFeatures.base.x);
% baseXmax = max(mapFeatures.base.x);
% baseYmin = min(mapFeatures.base.y);
% baseYmax = max(mapFeatures.base.y);
% 
% % Choose a random edge of the base perimeter to be the goal
% baseEdge = randi(4);
% switch baseEdge
%     case 1 % Top edge
%         goal(1) = randi([baseXmin, baseXmax]);
%         goal(2) = baseYmax;
%         goal(3) = start(3); % Keep the same heading as the start
%     case 2 % Bottom edge
%         goal(1) = randi([baseXmin, baseXmax]);
%         goal(2) = baseYmin;
%         goal(3) = start(3); % Keep the same heading as the start
%     case 3 % Left edge
%         goal(1) = baseXmin;
%         goal(2) = randi([baseYmin, baseYmax]);
%         goal(3) = start(3); % Keep the same heading as the start
%     case 4 % Right edge
%         goal(1) = baseXmax;
%         goal(2) = randi([baseYmin, baseYmax]);
%         goal(3) = start(3); % Keep the same heading as the start
%     otherwise % Prints warning if no case is satisfied
%         warning('Error in base goal assignment from switch cases');
% end

% Vehicle Parameters
% speed = 3; % Vehicle speed
% R = 8; % Minimum turn radius

% Other defined variables for function
% DT = 1; % Timestep
L = speed*DT; % Step length
dtr = pi/180; % Degrees to radian
cost_discretization = 1; % Number of intermediate points for plotting
relaxload = 0.3; % Relaxation parameter

% Define the domain (map)
global domain;
N = 3;  %number of dimensions, (x,y,heading)
domain = [xmin, xmax; ymin, ymax; 0, 2*pi]; % [xmap length, ymap length, startangle]
dx = L; dy = L; dth = L/R;

% %plot the domain
% figure(1);
% drawdomain(domain, 'k', N);
% hold on;

%%% Obstacles and Load Functions

% Define the obstacles from the mapFeatures structure variable
obstacles = mapFeatures.obstacles;

% Define load function
loadfunc = struct();
loadfunc.number = 0; %number of loading functions

% Load function evaluation using gaussian distribution, can also use pre-computed tablef
loadfunc.thres{1} = 10.0;
loadfunc.location{1} = [0, -5000]; %yeet that thing off the map
ang = 150*dtr; %change the orientation of the gaussian distribution
S = [cos(ang), sin(ang); -sin(ang) cos(ang)];
loadfunc.shape{1} = inv(S*[200 0; 0 500]*S');
loadfunc.eval{1} = @(x,y) exp(-0.5*([x, y] - loadfunc.location{1})*loadfunc.shape{1}*([x, y] - loadfunc.location{1})');

% Define spatial cost function
costfunc = @(x,y) speed;

%%% Set up the grid. note: the start location snaps on to the grid, but the
%%% goal location has no guarantee of doing so.
% X--
grid_x_left = start(1):-dx:domain(1,1);
grid_x_right = start(1):dx:domain(1,2);
grid_x = [fliplr(grid_x_left), grid_x_right(2:end)];
LX = length(grid_x);
% Y--
grid_y_down = start(2):-dy:domain(2,1);
grid_y_up = start(2):dy:domain(2,2);
grid_y = [fliplr(grid_y_down), grid_y_up(2:end)];
LY = length(grid_y);

% THETA--
grid_th_down = start(3):-dth:domain(3,1);
grid_th_up = start(3):dth:domain(3,2);
grid_th = [fliplr(grid_th_down), grid_th_up(2:end)];
LTH = length(grid_th);

if LTH < 2 %debug
    error('Turn angle is too low, causing inadequate heading discretization');
end

% redefine domain
domain = [ grid_x(1) grid_x(LX); grid_y(1) grid_y(LY); grid_th(1) grid_th(LTH)];

% Compute start and goal index
start_index = [length(grid_x_left), length(grid_y_down), length(grid_th_down)];

[~, g_ind_x] = min(abs(grid_x - goal(1)));
[~, g_ind_y] = min(abs(grid_y - goal(2)));
[~, g_ind_th] = min(abs(grid_th - goal(3)));
goal_index = [g_ind_x, g_ind_y, g_ind_th];

start_indexc = (start_index(3) - 1)*LX*LY + (start_index(2) - 1)*LX + start_index(1);
goal_indexc = (goal_index(3) - 1)*LX*LY + (goal_index(2) - 1)*LX + goal_index(1);

% Number of nodes in the grid
numgraph = LX*LY*LTH;

%%% Plot the Start and Goal & mesh the x-y (good for plotting)
[grid_mesh_x, grid_mesh_y] = meshgrid(grid_x, grid_y);

figure(1)
%plot(grid_mesh_x, grid_mesh_y, 'kx', 'Markersize', 1);
plot(start(1), start(2), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
quiver(start(1), start(2), L*cos(start(3))*3, L*sin(start(3))*3, 'g', 'linewidth', 1);

% Plot start pose
plot(goal(1), goal(2), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');
quiver(goal(1), goal(2), L*cos(goal(3))*3, L*sin(goal(3))*3, 'r', 'linewidth', 1);

% Plot goal pose
plot(grid_x(goal_index(1)), grid_y(goal_index(2)), 'ms', 'Markersize', 6, 'Markerfacecolor', 'r');
quiver(grid_x(goal_index(1)), grid_y(goal_index(2)), L*cos(grid_th(goal_index(3)))*5, L*sin(grid_th(goal_index(3)))*5, 'm', 'linewidth', 1);

% Plot the obstacles and the load functions
plot_loadfunc(loadfunc, grid_mesh_x, grid_mesh_y, figure(1));
plot_obstacles(obstacles, figure(1));

start_index = start_indexc;
goal_index = goal_indexc;

% Plot the output
plot_steps = 2;

fprintf("Parameters Set, Running Search \n\n");

%% Backtracking Hybrid A* Graph Search %%

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
    if isempty(actions) %debug
        error('No valid actions generated, theta might be too low');
    end
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

plot(path(:,1),path(:,2),'o','Color','blue')

fprintf("Terminal Cost: %f\n", current.cost);
fprintf("Terminal Load: %f\n", current.load);
fprintf("Runtime: %f s\n", runtime);


end %function end
