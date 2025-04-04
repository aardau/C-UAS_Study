function MCData = calculateMonteCarloData(MCData, N)
% Updates the Monte Carlo statistics for each iteration N

    % Calculate defense success rate
    defenseRate = sum(MCData.killVar(1:N)) / N * 100;
    MCData.successRateHistory(N) = defenseRate;

    % Calculate standard deviation based on all completed simulations
    MCData.stdSuccessRate(N) = std(MCData.successRateHistory(1:N));

    % Calculate Clopper-Pearson confidence interval using binofit
    [phat, pci] = binofit(sum(MCData.killVar(1:N)), N, 0.05);  % 95% confidence interval

    % Convert confidence interval percentages and update MCData
    MCData.ciHistory(N) = phat * 100;
    MCData.lowerCIHistory(N) = pci(1) * 100;
    MCData.upperCIHistory(N) = pci(2) * 100;

end % Function end