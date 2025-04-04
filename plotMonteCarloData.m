function [MCData, plotHandles] = plotMonteCarloData(maxIterations)
% Initializes Monte Carlo analysis data and their plots

    % Initialize Monte Carlo data and put into a structure var
    MCData.maxIterations = maxIterations;
    MCData.killVar = zeros(maxIterations, 1);
    MCData.killXY = NaN(maxIterations, 2);
    MCData.successRateHistory = zeros(maxIterations, 1);
    MCData.stdSuccessRate = zeros(maxIterations, 1);
    MCData.ciHistory = zeros(maxIterations, 1);
    MCData.lowerCIHistory = zeros(maxIterations, 1);
    MCData.upperCIHistory = zeros(maxIterations, 1);

    % Initialize success rate plot (Figure 2)
    figure(2);
    plotHandles.successPlot = plot(nan, nan, 'o-', 'LineWidth', 1);
    xlabel('Number of Simulations');
    ylabel('Defense Success Rate (%)');
    title('Cumulative Defense Success Rate');
    ylim([0 100]);
    grid on;
    hold on;

    % Initialize standard deviation plot (Figure 3)
    figure(3);
    plotHandles.stdPlot = plot(nan, nan, 'o-', 'LineWidth', 1);
    xlabel('Number of Simulations');
    ylabel('Standard Deviation of Success Rate (%)');
    title('Standard Deviation of Defense Success Rate');
    grid on;

    % Initialize confidence interval plot (Figure 4)
    figure(4);
    plotHandles.ciErrorBar = errorbar(nan, nan, nan, nan, 'o-', 'LineWidth', 1);
    xlabel('Number of Simulations');
    ylabel('Defense Success Rate (%)');
    title('Clopper-Pearson Confidence Interval');
    ylim([0 100]);
    grid on;

end % Function end