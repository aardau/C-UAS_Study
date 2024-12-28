function plotSimulation(x, y, fixedPoint)
    % Plot UAV path on the map
    plot(x, y, 'r', 'LineWidth', 2);
    hold on;
    plot(fixedPoint(1), fixedPoint(2), 'bs', 'MarkerSize', 10);
    title('UAV Simulation');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
end