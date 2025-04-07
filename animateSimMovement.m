function animateSimMovement(uasPath, mobilePath, mobileRange, pauseTime, killTimeStep, gifFilename, iteration)
% Animates UAS and mobile defense movements (with trails and a mobile defense range)

    figure(1);
    hold on;
    
    % Clear previous dynamic objects for this iteration
    delete(findobj(gca, 'Tag', 'UASTrail'));
    delete(findobj(gca, 'Tag', 'MobileTrail'));
    
    % Update title to show the current iteration number
    title(sprintf('UAS Simulation - Iteration %d', iteration));
    
    % Create animated lines for trails with tags
    uasTrail = animatedline('Tag','UASTrail','Color','m','LineWidth',1.5);
    mobileTrail = animatedline('Tag','MobileTrail','Color','c','LineWidth',1.5);
    
    % Initialize markers for current positions
    uasMarker = plot(uasPath(1,1), uasPath(1,2), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'Tag', 'UASTrail');
    mobileMarker = plot(mobilePath(1,1), mobilePath(1,2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'Tag', 'UASTrail');
    
    % Create mobile defense range patch
    theta = linspace(0, 2*pi, 100);
    xCircle = mobileRange * cos(theta) + mobilePath(1,1);
    yCircle = mobileRange * sin(theta) + mobilePath(1,2);
    mobileRangePatch = patch(xCircle, yCircle, 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'b','Tag', 'UASTrail');
    
    % Determine number of frames for this iteration
    numFrames = max(size(uasPath,1), size(mobilePath,1));
    killMarkerDisplayed = false;
    
    % Persistent flag to check if the GIF has been initialized
    persistent gifInitialized
    if isempty(gifInitialized)
        gifInitialized = false;
    end
    
    for t = 1:numFrames
        % Update UAS trail and marker
        if t <= size(uasPath,1)
            addpoints(uasTrail, uasPath(t,1), uasPath(t,2));
            if ~isnan(killTimeStep) && t >= killTimeStep
                if isvalid(uasMarker)
                    delete(uasMarker);
                end
                if ~killMarkerDisplayed
                    plot(uasPath(t,1), uasPath(t,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
                    killMarkerDisplayed = true;
                end
            else
                if isvalid(uasMarker)
                    set(uasMarker, 'XData', uasPath(t,1), 'YData', uasPath(t,2));
                end
            end
        end
        
        % Update mobile defense trail and marker
        if t <= size(mobilePath,1)
            addpoints(mobileTrail, mobilePath(t,1), mobilePath(t,2));
            set(mobileMarker, 'XData', mobilePath(t,1), 'YData', mobilePath(t,2));
            xCircle = mobileRange * cos(theta) + mobilePath(t,1);
            yCircle = mobileRange * sin(theta) + mobilePath(t,2);
            set(mobileRangePatch, 'XData', xCircle, 'YData', yCircle);
        end
        
        drawnow;
        pause(pauseTime);
        
        % Capture frame and write to GIF
        frame = getframe(figure(1));
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if ~gifInitialized
            imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', pauseTime);
            gifInitialized = true;
        else
            imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', pauseTime);
        end
    end
end