
%%%Parameter Script for Backtracking Hybrid-A*%%%
%set up the workspace to run path planning algorithms

global EPS;
EPS = 1e-6;

dtr = pi/180; %degrees to radians

%%%Vehicle Parameters
speed = 3; %vehicle speed
R = 8; %turn radius

DT = 1; %timestep
L = speed*DT; %step length

cost_discretization = 8;

%%% Domain
global domain;
N = 3;  %number of dimensions
domain = [0 100; 0, 100; 0, 2*pi]; %domain size, in search coords

dx = L; dy = L; dth = L/R;

%plot the domain
figure(1);
drawdomain(domain, 'k', N);
hold on;

%%%Obstacles and Load Function

%Obstacles
obstacles = struct();
obstacles.number = 3;
obstacles.type = 1; % AABB - 0, Closed Polygon - 1
obstacles.vertices{1} = [20,30; 40,50; 30,90; 25,85];
obstacles.vertices{2} = [50,25; 80,25; 80,45; 50,40]; 
obstacles.vertices{3} = [11,2; 11,20; 37 30; 39 2];

%Load Function
loadfunc = struct();
loadfunc.number = 1; %number of loading functiuons

%load evaluation
%currently using gaussian distribution, can also use pre-computed tablef
loadfunc.thres{1} = 10.0;
loadfunc.location{1} = [69, 25];
ang = 150*dtr; %change the orientation of the gaussian distrivbution
S = [cos(ang), sin(ang); -sin(ang) cos(ang)];
loadfunc.shape{1} = inv(S*[200 0; 0 500]*S');
loadfunc.eval{1} = @(x,y) exp(-0.5*([x, y] - loadfunc.location{1})*loadfunc.shape{1}*([x, y] - loadfunc.location{1})');

%spatial cost function
costfunc = @(x,y) speed;

%%%Start and Goal
start = [5.64516129032258,42.9620991253644,5.3];
goal = [80.0691244239632,55.2478134110787,1.35105099502532];

%relaxation parameter
relaxload = 0.3;

%%%Set up the grid: note: the start location snaps on to the grid, but the
%%%goal location has no guarantee of doing so.
%X--
grid_x_left = start(1):-dx:domain(1,1);
grid_x_right = start(1):dx:domain(1,2);
grid_x = [fliplr(grid_x_left), grid_x_right(2:end)];
LX = length(grid_x);
%Y--
grid_y_down = start(2):-dy:domain(2,1);
grid_y_up = start(2):dy:domain(2,2);
grid_y = [fliplr(grid_y_down), grid_y_up(2:end)];
LY = length(grid_y);

%%%THETA--
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

%%%Plot the Obstacles and The Load Functions
plot_loadfunc(loadfunc, grid_mesh_x, grid_mesh_y, figure(1));
plot_obstacles(obstacles, figure(1));

start_index = start_indexc;
goal_index = goal_indexc;

%plot output
plot_steps = 2;

fprintf("Parameters Set, Running Search \n\n");

