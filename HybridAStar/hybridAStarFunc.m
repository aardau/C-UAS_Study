% Desc

function [path] = hybridAStarFunc(mapBounds,mapFeatures, velocity,maxTheta,timestep)
%%% Parameter Section for Backtracking Hybrid-A* %%%

% Extract map bounds for calculations
xmin = mapBounds(1); xmax = mapBounds(2);
ymin = mapBounds(3); ymax = mapBounds(4);

% Vehicle parameters (unnecessary to redefine but notes their existence)
vel = velocity; % Vehicle speed
theta = maxTheta; % Turn radius
dT = timestep; %timestep

% Initialize start and end goal
start = [0,0,0];
goal = [0,0,0]; % Can keep as origin for now

% Choose a random edge of the map to spawn the UAS
edge = randi(4);
switch edge
    case 1 % Top edge
        start(1) = randi([xmin, xmax]);
        start(2) = ymax;
        start(3) = 3*pi/2;
        goal(3) = start(3); % Keep goal heading as start heading
    case 2 % Bottom edge
        start(1) = randi([xmin, xmax]);
        start(2) = ymin;
        start(3) = pi/2;
        goal(3) = start(3); % Keep goal heading as start heading
    case 3 % Left edge
        start(1) = xmin;
        start(2) = randi([ymin, ymax]);
        start(3) = 0;
        goal(3) = start(3); % Keep goal heading as start heading
    case 4 % Right edge
        start(1) = xmax;
        start(2) = randi([ymin, ymax]);
        start(3) = pi;
        goal(3) = start(3); % Keep goal heading as start heading
    otherwise % Prints warning if no case is satisfied
        warning('Error in UAS spawning from switch cases')
end

% Define the domain
N = 3;  %number of dimensions
%domain size, in search coords
domain = [xmin, xmax; ymin, ymax; 0, 2*pi]; % [xmap length, ymap length, startangle]
% Other defined variables for function
cost_discretization = 8;
L = vel*dT; %step length
dx = L; dy = L; dth = L/theta;

% Define the obstacles from the mapFeatures structure variable
obstacles = mapFeatures.obstacles;

%spatial cost function
costfunc = @(x,y) vel;

% Set up the grid: note: the start location snaps on to the grid, but the 
% goal location has no guarantee of doing so.
% X--
grid_x_left = start(1):-dx:domain(1,1);
grid_x_right = start(1):dx:domain(1,2);
grid_x = [fliplr(grid_x_left), grid_x_right(2:end)];
LX = length(grid_x);

%Y --
grid_y_down = start(2):-dy:domain(2,1);
grid_y_up = start(2):dy:domain(2,2);
grid_y = [fliplr(grid_y_down), grid_y_up(2:end)];
LY = length(grid_y);

% THETA--
grid_th_down = start(3):-dth:domain(3,1);
grid_th_up = start(3):dth:domain(3,2);
grid_th = [fliplr(grid_th_down), grid_th_up(2:end)];
LTH = length(grid_th);
%%%redefine domain
domain = [ grid_x(1) grid_x(LX); grid_y(1) grid_y(LY); grid_th(1) grid_th(LTH)];

%%%Compute Start and Goal Index
start_index = [length(grid_x_left), length(grid_y_down), length(grid_th_down)];

[~, g_ind_x] = min(abs(grid_x - goal(1)));
[~, g_ind_y] = min(abs(grid_y - goal(2)));
[~, g_ind_th] = min(abs(grid_th - goal(3)));
goal_index = [g_ind_x, g_ind_y, g_ind_th];

start_indexc = (start_index(3) - 1)*LX*LY + (start_index(2) - 1)*LX + start_index(1);
goal_indexc = (goal_index(3) - 1)*LX*LY + (goal_index(2) - 1)*LX + goal_index(1);

%number of nodes in the grid
numgraph = LX*LY*LTH;

%%%Plot the Start and Goal
%mesh the x-y: good for plotting
[grid_mesh_x, grid_mesh_y] = meshgrid(grid_x, grid_y);

figure(1)
plot(grid_mesh_x, grid_mesh_y, 'kx', 'Markersize', 1);
plot(start(1), start(2), 'go', 'Markersize', 6, 'Markerfacecolor', 'g');
quiver(start(1), start(2), L*cos(start(3))*3, L*sin(start(3))*3, 'g', 'linewidth', 1);

%start pose
plot(goal(1), goal(2), 'ro', 'Markersize', 6, 'Markerfacecolor', 'r');
quiver(goal(1), goal(2), L*cos(goal(3))*3, L*sin(goal(3))*3, 'r', 'linewidth', 1);

%goal pose
plot(grid_x(goal_index(1)), grid_y(goal_index(2)), 'ms', 'Markersize', 6, 'Markerfacecolor', 'r');
quiver(grid_x(goal_index(1)), grid_y(goal_index(2)), L*cos(grid_th(goal_index(3)))*5, L*sin(grid_th(goal_index(3)))*5, 'm', 'linewidth', 1);

start_index = start_indexc;
goal_index = goal_indexc;

%plot output
plot_steps = 2;

%% Backtracking Hybrid A* Graph Search

reached_goal = 0;

tic

num_actions = 3;

%frontier structure
frontier = struct();
frontier.cost = 0;
frontier.starcost = 0;
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

%child vals => -1 in frontier, 0 > in visited, 0 doesnt exist

while ~isempty(frontier.cost)
    %assign the current node
    current = struct();
    current.cost = frontier.cost(1);
    current.starcost = frontier.starcost(1);
    current.idx = frontier.idx(1);
    current.node = frontier.node(1, :);
    current.parent = frontier.parent(1);
    current.action = frontier.action(1);
    
    frontier.cost = frontier.cost(2:end);
    frontier.starcost = frontier.starcost(2:end);
    frontier.idx = frontier.idx(2:end);
    frontier.node = frontier.node(2:end, :);
    frontier.parent = frontier.parent(2:end);
    frontier.action = frontier.action(2:end);

    %pop the node if it has been visited
    if visited(current.idx) == 1
        continue;
    end
    
    %add the node to the visited set
    visited(current.idx) = 1;
    parent(current.idx) = current.parent;
    parent_act(current.idx) = current.action;
    node(current.idx, :) = current.node;
    cost(current.idx) = current.cost;
    
    if current.idx ~= start_index
        child(current.parent, current.action) = current.idx;
    end
    
    if current.idx == goal_index
        reached_goal = 1;
        break
    end
    
    actions = get_actions_kinem(current.node, L, theta, grid_x, grid_y, grid_th);
    for aci = 1:num_actions
        newnode = struct();
        newnode.node = actions.newlocation(aci, :);
        newnode.idx = actions.newindexc(aci);
        newnode.parent = current.idx;

        node_admissibility = check_admissibility(obstacles, newnode, current, theta, aci, domain);
        
        %check if the node is valid
        if ~node_admissibility
            continue;
        end

        if visited(newnode.idx) > 0 
            continue;
        end
        
        %compute the node cost using the cost function
        candpath = drawaction(current.node, L, theta, aci, cost_discretization);
        new_cost = integrate_cost(candpath, vel, costfunc);
        
        newnode.cost = current.cost + new_cost;
        newnode.starcost = newnode.cost + norm(newnode.node(1:2) - goal(1:2));
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
        action = drawaction(path(i,:), L, theta, path_aci(i+1), cost_discretization);
        final = plot(action(:,1), action(:,2), 'LineWidth', 3, 'Color', 'magenta');
    end
end

%fprintf("Terminal Cost: %f\n", current.cost);
fprintf("Hybrid A* Runtime: %f s\n", runtime);

%% Drawaction function

function actionpath = drawaction(current, L, R, actionum, cost_discretization)

    %%%in this one: there are only three possible actions: move straight by a
    %%%fixed length, left or right by the same length at the vehicles min turn
    %%%radius

    %%%current pose
    Sx = current(1);
    Sy = current(2);
    Sth = current(3);

    actionpath = current;
    turnangle = L/R;
    disclen = cost_discretization;   %number of intermediate points for plotting
    %%%center of turn: turning "up"
    if actionum == 0    %do nothing
        actionpath = repmat(current, disclen, 1);
    elseif actionum == 1    %move straight
        lendisc = linspace(0,L,disclen);
        for i = 1:disclen
            actionpath(1+i, :) = [Sx + lendisc(i)*cos(Sth), Sy + lendisc(i)*sin(Sth), Sth];
        end        
    elseif actionum == 2    %turn "up"
        Cxup = Sx - R*sin(Sth);
        Cyup = Sy + R*cos(Sth);
        newangle_up = Sth + turnangle;
        angledisc = linspace(Sth, newangle_up, disclen);
        for i = 1:disclen
            actionpath(1+i,:) = [Cxup + R*sin(angledisc(i)), Cyup - R*cos(angledisc(i)), mod(angledisc(i), 2*pi)];
        end    
    else   %turn down
        Cxdn = Sx + R*sin(Sth);
        Cydn = Sy - R*cos(Sth);
        newangle_down = Sth - turnangle;
        angledisc = linspace(Sth, newangle_down, disclen);
        for i = 1:disclen
            actionpath(1+i,:) = [Cxdn + R*sin(-angledisc(i)), Cydn + R*cos(-angledisc(i)), mod(angledisc(i), 2*pi)];
        end    
    end
end

%% check_admissibility function

function [adm] = check_admissibility(obstacles, newnode, current, turnrad, aci, domain)
    N = 2;

    %%%check solution domain admissibility
    admissdomain = true(size(current.node,1), 1);
    for i = 1:N
        in = newnode.node(:, i) >= domain(i,1) & newnode.node(:,i) <= domain(i,2);
        admissdomain = admissdomain & in;
    end
    adm = admissdomain;
    
    n = aci; %need to pass type of action being performed (S U D)
    obsnum = obstacles.number;
    switch n
        case 2 %if the action is turning up 
            u = [-sin(current.node(3)) cos(current.node(3))]*turnrad; %vector ccw perpendicular to current pose * turnrad
            p1 = current.node(1:2); 
            d1 = newnode.node(1:2)-p1;
            c = p1 + u; % center of the turn arc
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p2 = verts(j,:); %start and endpoint for edge
                    d2 = verts(j+1,:)-p2;
                    % checking for the intersection
                    delta = p2 - c;
                    cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - turnrad^2); %if roots are not complex the line and the circle intersect
                    if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                        t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                        t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                        if t1 >= 0 && t1 <= 1
                            p = p2 + t1*d2;
                            A = p-p1;
                            B = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end
                        if t2 >= 0 && t2 <= 1 
                            p = p2 + t2*d2;
                            A = p-p1;
                            B = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end 
                    end
                end
            end 
            
        case 3 %if the action is turning up
            u = [sin(current.node(3)) -cos(current.node(3))]*turnrad; %vector ccw perpendicular to current pose * turnrad
            p1 = current.node(1:2);
            d1 = newnode.node(1:2)-p1;
            c = p1 + u; % center of the turn arc
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p2 = verts(j,:); %start and endpoint for edge
                    d2 = verts(j+1,:)-p2;
                    % checking for an intersection
                    delta = p2 - c;
                    cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - turnrad^2); %if roots are not complex the line and the circle intersect
                    if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                        t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                        t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                        if t1 >= 0 && t1 <= 1
                            p = p2 + t1*d2;
                            B = p-p1; %because turn down is going cw need to swap bounds of the arc
                            A = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end
                        if t2 >= 0 && t2 <= 1 
                            p = p2 + t2*d2;
                            B = p-p1; %because turn down is going cw need to swap bounds of the arc
                            A = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end 
                    end
                end
            end
            
        case 1 %if the action is going straight
            p0 = current.node(1:2); %start and end for path
            d0 = newnode.node(1:2)-p0;
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p1 = verts(j,:); %start and endpoint for edge
                    d1 = verts(j+1,:)-p1;
                    %checking for an intersection
                    denom = d0(1)*d1(2) - d1(1)*d0(2); % the lines intersect if this product is not zero
                    if denom ~= 0
                        delt = p1 - p0;
                        s = (delt(1)*d1(2) - d1(1)*delt(2))/denom;
                        t = (delt(1)*d0(2) - d0(1)*delt(2))/denom;
                        %if the lines intersect andd the point is on both
                        %the obstacle line and the path return and set adm
                        %to false
                        if (s>=0 && s<=1) && (t>=0 && t<=1) 
                            adm = 0;
                            return;
                        end
                    end
                end
            end 
        otherwise 
            fprintf('Error, invalid case.\n')
    end
end

%% get_actions_kinem function

function [actions] = get_actions_kinem(current, L, R, grid_x, grid_y, grid_th)
    %%%current pose
    Sx = current(1);
    Sy = current(2);
    Sth = current(3);

    %%%center of turn: turning "up"
    Cxup = Sx - R*sin(Sth);
    Cyup = Sy + R*cos(Sth);
    Cxdn = Sx + R*sin(Sth);
    Cydn = Sy - R*cos(Sth);

    %%%turn angles
    turnangle = L/R;
    newangle_up = Sth + turnangle;
    newangle_down = Sth - turnangle;

    %%%movements: new locations: 1) forward, 2) turn "up", 3) turn "down"
    actions.newlocation = [Sx + L*cos(Sth), Sy + L*sin(Sth), Sth;
        Cxup + R*sin(newangle_up), Cyup - R*cos(newangle_up), mod(newangle_up, 2*pi);
        Cxdn + R*sin(-newangle_down), Cydn + R*cos(-newangle_down), mod(newangle_down, 2*pi)];    

    actions.newindex = zeros(3,3);
    LX_kinem = length(grid_x);
    LY_kinem = length(grid_y);
    
    for i = 1:3
        location = actions.newlocation(i,:);
        
        xdiff = abs(grid_x - location(1));
        [~, xpmin] = min(xdiff);
        if xpmin == 1 &&  (grid_x(1) >  location(1))
            actions.newindex(i,1) = -inf;    %driven off the domain
        elseif xpmin == LX_kinem &&  (grid_x(LX_kinem) <  location(1))
            actions.newindex(i,1) = +inf;  %driven off the domain
        else
            actions.newindex(i,1) = xpmin;
        end
        
        ydiff = abs(grid_y - location(2));
        [~, ypmin] = min(ydiff);
        if ypmin == 1 &&  (grid_y(1) >  location(2))
            actions.newindex(i,2) = -inf;    %driven off the domain
        elseif ypmin == LY_kinem &&  (grid_y(LY_kinem) <  location(2))
            actions.newindex(i,2) = inf;  %driven off the domain
        else
            actions.newindex(i,2) = ypmin;        
        end
        
        %%%the th-index is cyclic. It will never drive off the domain
        thdiff = abs(grid_th - location(3));
        [~, thpmin] = min(thdiff);
        actions.newindex(i,3) = thpmin;
    end

    %%%obtain cumulative index
    for i = 1:3
        actions.newindexc(i) = (actions.newindex(i,3) - 1)*LX_kinem*LY_kinem + (actions.newindex(i,2) - 1)*LX_kinem + actions.newindex(i,1);
    end

end

%% integrate_cost function

function [cost] = integrate_cost(candpath, speed, cost_func)
    
    N = size(candpath, 1);
    cost = 0;
    
    %center numerical integration
    for i = 1:N-1
        dc = (cost_func(candpath(i,1), candpath(i,2)) + cost_func(candpath(i+1,1), candpath(i+1,2)))/2;
        dt = norm(candpath(i, 1:2) - candpath(i+1, 1:2))/speed;
        cost = cost + dc*dt;
    end
    
end

end

