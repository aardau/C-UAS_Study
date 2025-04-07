function plotStaticMapFeatures(mapBounds, mapFeatures)
% Plots the run-independent map features once and before the Monte Carlo
% looping

% Figure 1 is the simulated environment
figure(1);
hold on

% Adjust plot settings for figure 1
xlim([mapBounds(1), mapBounds(2)]);
ylim([mapBounds(3), mapBounds(4)]);
xlabel('X Position (units)');
ylabel('Y Position (units)');
grid on;

% Plot base
base = polyshape(mapFeatures.base.x, mapFeatures.base.y);
plot(base);

% Plot map obstacles
obstacles = mapFeatures.obstacles;  % Extract obstacles from mapFeatures
xO = []; % Initialize X vals
yO = []; % Initialize y vals
if isfield(mapFeatures, 'obstacles') && mapFeatures.obstacles.number > 0
    for k = 1:obstacles.number
        xO = [xO; obstacles.vertices{k}(:,1); obstacles.vertices{k}(1,1); NaN];
        yO = [yO; obstacles.vertices{k}(:,2); obstacles.vertices{k}(1,2); NaN];
        fill(obstacles.vertices{k}(:,1), obstacles.vertices{k}(:,2), 'k'); % Fill obstacles in black
    end
    plot(xO, yO, 'Color', [0.2, 0.2, 1], 'LineWidth', 2);
end

%Plot static defenses
for i = 1:height(mapFeatures.staticDefenses)
    x = mapFeatures.staticDefenses(i, 1) - mapFeatures.staticDefenses(i, 3):1:mapFeatures.staticDefenses(i, 1) + mapFeatures.staticDefenses(i, 3);
    R = mapFeatures.staticDefenses(i, 3);
    y1 = sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
    y2 = -sqrt(R^2 - (x - mapFeatures.staticDefenses(i, 1)).^2) + mapFeatures.staticDefenses(i, 2);
    plot(x, y1, x, y2,'Color',"r");
end

%Plot mobile defenses
for j = 1:height(mapFeatures.mobileDefenses)
    x = mapFeatures.mobileDefenses(j, 1) - mapFeatures.mobileDefenses(j, 3):1:mapFeatures.mobileDefenses(j, 1) + mapFeatures.mobileDefenses(j, 3);
    R = mapFeatures.mobileDefenses(j, 3);
    y1 = sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
    y2 = -sqrt(R^2 - (x - mapFeatures.mobileDefenses(j, 1)).^2) + mapFeatures.mobileDefenses(j, 2);
    plot(x, y1, x, y2,'Color',"b");
end

end % Function end