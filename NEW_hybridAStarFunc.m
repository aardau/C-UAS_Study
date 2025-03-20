function [path] = NEW_hybridAStarFunc(mapBounds,mapFeatures, speed,R,DT)

% Extract map bounds for calculations
xmin = mapBounds(1); xmax = mapBounds(2);
ymin = mapBounds(3); ymax = mapBounds(4);

% Initialize start and end goal
start = [0,0,0];
goal = [0,0,0]; 

% Initialize path
path = [];

% Choose a random edge of the map to spawn the UAS
edge = randi(4);
switch edge
    case 1 % Top edge spawn
        start(1) = randi([xmin, xmax]);
        start(2) = ymax;
        start(3) = 3*pi/2;
        goal(3) = start(3); % Keep the same heading as the start
    case 2 % Bottom edge spawn
        start(1) = randi([xmin, xmax]);
        start(2) = ymin;
        start(3) = pi/2;
        goal(3) = start(3); % Keep the same heading as the start
    case 3 % Left edge spawn
        start(1) = xmin;
        start(2) = randi([ymin, ymax]);
        start(3) = 0;
        goal(3) = start(3); % Keep the same heading as the start
    case 4 % Right edge spawn
        start(1) = xmax;
        start(2) = randi([ymin, ymax]);
        start(3) = pi;
        goal(3) = start(3); % Keep the same heading as the start
    otherwise % Prints warning if no case is satisfied
        warning('Error in UAS spawning from switch cases')
end

%fprintf('Debug: Goal positions initialized at (%.2f, %.2f, %.2f)\n', goal(1), goal(2), goal(3));

% Extract base boundaries for goal setting
baseXmin = min(mapFeatures.base.x);
baseXmax = max(mapFeatures.base.x);
baseYmin = min(mapFeatures.base.y);
baseYmax = max(mapFeatures.base.y);

% Use the same spawn edge to determine the goal location along the base perimeter
switch edge
    case 1 % Top edge spawn --> goal on top wall
        goal(1) = randi([baseXmin, baseXmax]);
        goal(2) = baseYmax;
        goal(3) = start(3);
    case 2 % Bottom edge spawn --> goal on bottom wall
        goal(1) = randi([baseXmin, baseXmax]);
        goal(2) = baseYmin;
        goal(3) = start(3);
    case 3 % Left edge spawn --> goal on left wall
        goal(1) = baseXmin;
        goal(2) = randi([baseYmin, baseYmax]);
        goal(3) = start(3);
    case 4 % Right edge spawn --> goal on right wall
        goal(1) = baseXmax;
        goal(2) = randi([baseYmin, baseYmax]);
        goal(3) = start(3);
    otherwise
        warning('Error in base goal assignment from switch cases');
end

% Check if goal is within bounds
if goal(1) < xmin || goal(1) > xmax || goal(2) < ymin || goal(2) > ymax
    warning('Goal coordinates are out of map bounds: [%f, %f]', goal(1), goal(2));
end

% Other defined variables for function
L = speed*DT; % Step length
dtr = pi/180; % Degrees to radian
cost_discretization = 1; % Number of intermediate points for plotting
relaxload = 1; % Relaxation parameter

% Define the domain (map)
global domain; %code fails without global var
N = 3;  %number of dimensions, (x,y,heading)
domain = [xmin, xmax; ymin, ymax; 0, 2*pi]; % [xmap length, ymap length, startangle]
dx = L; dy = L; dth = L/R;

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

% Check if turn angle is too low
if LTH < 2
    warning('Turn angle is too low, causing inadequate heading discretization');
end

% Verify grid values
if isempty(grid_x) || isempty(grid_y) || isempty(grid_th)
    warning('Grid arrays are empty! Check domain setup.');
end

% redefine domain
domain = [ grid_x(1) grid_x(LX); grid_y(1) grid_y(LY); grid_th(1) grid_th(LTH)];

% Compute start and goal index
start_index = [length(grid_x_left), length(grid_y_down), length(grid_th_down)];

[~, g_ind_x] = min(abs(grid_x - goal(1)));
[~, g_ind_y] = min(abs(grid_y - goal(2)));
[~, g_ind_th] = min(abs(grid_th - goal(3)));
goal_index = [g_ind_x, g_ind_y, g_ind_th];

%fprintf('Debug: Goal indices: X = %d, Y = %d, Theta = %d\n', g_ind_x, g_ind_y, g_ind_th);

% % Redefine goals to nearest grid values
% goal(1) = grid_x(g_ind_x);
% goal(2) = grid_y(g_ind_y);
% goal(3) = grid_th(g_ind_th);

%fprintf('Debug: Goal position after snapping to grid (%.2f, %.2f, %.2f)\n', goal(1), goal(2), goal(3));

start_indexc = (start_index(3) - 1)*LX*LY + (start_index(2) - 1)*LX + start_index(1);
goal_indexc = (goal_index(3) - 1)*LX*LY + (goal_index(2) - 1)*LX + goal_index(1);


% Number of nodes in the grid
numgraph = LX*LY*LTH;

% Check computed indices
if start_indexc < 1 || start_indexc > numgraph
    warning('Start index out of bounds: %d (numgraph = %d)', start_indexc, numgraph);
end
if goal_indexc < 1 || goal_indexc > numgraph
    warning('Goal index out of bounds: %d (numgraph = %d)', goal_indexc, numgraph);
end

% Check if goal index is valid
if g_ind_x < 1 || g_ind_x > length(grid_x) || g_ind_y < 1 || g_ind_y > length(grid_y) || g_ind_th < 1 || g_ind_th > length(grid_th)
    warning('Goal index is out of bounds: [%d, %d, %d]', g_ind_x, g_ind_y, g_ind_th);
end

%fprintf('Debug: Goal index computed as %d, valid range [1, %d]\n', goal_indexc, numgraph);

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

%fprintf("Parameters Set, Running Search \n\n");

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
    
    %fprintf('Debug: current.idx = %d, goal_index = %d\n', current.idx, goal_index);

    actions = get_actions_kinem(current.node, L, R, grid_x, grid_y, grid_th);


    for aci = 1:num_actions
        newnode = struct();
        newnode.node = actions.newlocation(aci, :);
        newnode.idx = actions.newindexc(aci);
        newnode.parent = current.idx;

        node_admissibility = check_admissibility(obstacles, newnode, current, R, aci);
        
        % if ~node_admissibility (spams command window)
        %     fprintf('Node idx %d rejected due to admissibility check\n', newnode.idx);
        % end

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

%fprintf('Debug: plot_steps = %d, reached_goal = %d\n', plot_steps, reached_goal);

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
        %final = plot(action(:,1), action(:,2), 'LineWidth', 3, 'Color', 'magenta');
    end
else
    warning('(plot_steps > 0) && (reached_goal == true) not satisfied to create plot!')
end

% fprintf("Terminal Cost: %f\n", current.cost);
% fprintf("Terminal Load: %f\n", current.load);
fprintf("Hybrid A* Runtime: %f s\n", runtime);

%debug
if ~exist('path', 'var') || isempty(path)
    warning('Path var not created! Error with Hybrid A* function');
    path = [];
end

end %function end
