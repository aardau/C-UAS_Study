function [killVar, killTimeStep, kxy] = killCheck(SD, MD, trackProbability, killProbability, MF, UASP)
%Iterates throught the arrarys displaying whether or not the UAS was in
%range. Determines if the track is established and if a kill is achieved.
%Currently, the number of tracks needed is hard coded at 3

% initialize counting variables
trackCount = 0;
killVar = 0;
killTimeStep = NaN;
x = NaN; y = NaN;

% iterate throught entire height (over time)
for i = 1:height(SD) 
    for j = 1:width(SD) % check each static defense
        if SD(i, j) == 1
            chance = rand(1);
            if chance > (1-trackProbability)
                trackCount = trackCount+1;
            end
        end
    end
    for j = 1:width(MD) % check each mobile defense
        if MD(i, j) == 1
            chance = rand(1);
            if chance > (1-trackProbability)
                trackCount = trackCount+1;
            end
        end
    end
    % check if greater than 3 tracks have been established
    if trackCount >= 3
        chance = rand(1); % check if a kill attempt is succesful
        if chance > (1-killProbability)
            if killVar == 0
                killVar = 1;
                killTimeStep = i;
            end
        end
    end
end

% create simple vars
if ~isnan(killTimeStep)
    x = UASP(killTimeStep, 1); y = UASP(killTimeStep, 2);
    xb = MF.base.x;
    yb = MF.base.y;
    % check if kill was before the UAS reached base
    test = inpolygon(x, y, xb, yb);
    if test == 0
        fprintf("\nKill was succesfully outside of the base\n")
        fprintf("\nUAS killed at (%.0f, %.0f)\n", x, y)
    else
        fprintf("\nKill was achieved after the adversary reached the base\n")
        fprintf("\nUAS killed at (%.0f, %.0f)\n", x, y)
        killVar = 0;
    end
else
    fprintf("\nKill was not succesful\n")
end

kxy = [x; y];


end % EOF