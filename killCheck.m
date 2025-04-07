function [killVar, killTimeStep, kxy] = killCheck(SD, MD, staticTrackProb, mobileTrackProb, staticKillProb, mobileKillProb, MF, UASP)
% Iterates throught the arrarys displaying whether or not the UAS was in
% range. Determines if the track is established and if a kill is achieved.
% Currently, the number of tracks needed is hard coded at 3

% Initialize counting variables
staticCount = 0;
mobileCount = 0;
killVar = 0;
killTimeStep = NaN;
x = NaN; y = NaN;

% Iterate over time steps
for i = 1:height(SD)
    % Check static defenses
    for j = 1:width(SD)
        if SD(i,j) == 1
            if rand(1) > (1 - staticTrackProb)
                staticCount = staticCount + 1;
            end
        end
    end
    % Check mobile defenses
    for j = 1:width(MD)
        if MD(i,j) == 1
            if rand(1) > (1 - mobileTrackProb)
                mobileCount = mobileCount + 1;
            end
        end
    end
    
    % Check if static defenses have enough tracks to attempt a kill
    if staticCount >= 3
        if rand(1) > (1 - staticKillProb)
            killVar = 1;
            killTimeStep = i;
            break;  % Stop once a kill is achieved
        end
    end
    % Check if mobile defenses have enough tracks to attempt a kill
    if mobileCount >= 3
        if rand(1) > (1 - mobileKillProb)
            killVar = 1;
            killTimeStep = i;
            break;
        end
    end
end

% Determine the kill location from UAS path if a kill occurred
if ~isnan(killTimeStep)
    x = UASP(killTimeStep, 1);
    y = UASP(killTimeStep, 2);
    xb = MF.base.x;
    yb = MF.base.y;
    % Check if kill occurred before reaching base
    test = inpolygon(x, y, xb, yb);
    if test == 0
        fprintf("Kill was successfully outside of the base\n");
        fprintf("UAS killed at (%.0f, %.0f)\n\n", x, y);
    else
        fprintf("Kill was achieved after the adversary reached the base\n");
        fprintf("UAS killed at (%.0f, %.0f)\n\n", x, y);
        killVar = 0;  % Invalidate the kill if it occurs after reaching base
    end
else
    fprintf("Kill was not successful\n\n");
end

kxy = [x; y];

end % EOF