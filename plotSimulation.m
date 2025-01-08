function plotSimulation(x, y, fixedPoint)
    % Plot UAV path on the map
    plot(x, y, 'r', 'LineWidth', 2);
    hold on;
    plot(fixedPoint(1), fixedPoint(2), 'bs', 'MarkerSize', 10);
    %legend
    grid on;
end