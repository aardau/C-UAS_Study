function plotDynamicMapFeatures(N, MCData, plotHandles, uasToPlot, mobileToPlot, killTimeStep, MDHits, SDHits, mapFeatures)
% Updates dynamic plots and map features for each iteration

    % Update Monte Carlo plots

    % Success rate (Figure 2)
    set(plotHandles.successPlot, 'XData', 1:N, 'YData', MCData.successRateHistory(1:N));

    % Standard deviation (Figure 3)
    set(plotHandles.stdPlot, 'XData', 1:N, 'YData', MCData.stdSuccessRate(1:N));

    % Confidence interval (Figure 4)
    set(plotHandles.ciErrorBar, 'XData', 1:N, 'YData', MCData.successRateHistory(1:N), ...
        'LData', MCData.successRateHistory(1:N) - MCData.lowerCIHistory(1:N), ...
        'UData', MCData.upperCIHistory(1:N) - MCData.successRateHistory(1:N));

    % Refresh plots
    drawnow;

    % Update dynamic elements on Figure 1
    figure(1);
    hold on;
    % Update the title with the current iteration
    title(sprintf('UAS Simulation After %d Iterations', N));
    
    % Delete previous dynamic elements
    delete(findobj(gca, 'Tag', 'dynamic'));

    % Plot mobile defense movement if available
    if height(mapFeatures.mobileDefenses) > 0 && ~isempty(mobileToPlot)
        plot(mobileToPlot(:,1), mobileToPlot(:,2), 'c-', 'LineWidth', 1.5, 'Tag', 'dynamic');
        scatter(mobileToPlot(:,1), mobileToPlot(:,2), 10, 'c', 'filled', 'Tag', 'dynamic');
    end

    % Plot the UAS path
    plot(uasToPlot(:,1), uasToPlot(:,2), 'm-', 'LineWidth', 1.5, 'Tag', 'dynamic');
    scatter(uasToPlot(:,1), uasToPlot(:,2), 10, 'm', 'filled', 'Tag', 'dynamic');

    % Plot defense kill marker if a kill occurred
    if MCData.killVar(N) == 1 && ~isnan(killTimeStep)
        % Count hits from mobile and static defenses at the kill time step
        mobileHitCount = sum(MDHits(killTimeStep, :));
        staticHitCount = sum(SDHits(killTimeStep, :));

        % Choose marker color based on which defense contributed more hits
        if mobileHitCount >= staticHitCount
            plot(MCData.killXY(N, 1), MCData.killXY(N, 2), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'Tag', 'dynamic');
        else
            plot(MCData.killXY(N, 1), MCData.killXY(N, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'Tag', 'dynamic');
        end
    end
    
end