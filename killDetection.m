function [SDHits, MDHits] = killDetection(mapFeatures,uasPoints, mobileDefensePoints, SMD)
% The function takes in map features, the x-y pairs of the UAS, and x-y
% pairs of the mobile defense, as well as which mobile defense is moving
% towards the UAS to determine when the UAS is in range of each defense.

SDHits = zeros(height(uasPoints), height(mapFeatures.staticDefenses));
MDHits = zeros(height(uasPoints), height(mapFeatures.mobileDefenses));
if height(mapFeatures.staticDefenses>0)
    for i = 1:height(uasPoints)
        for j = 1:height(mapFeatures.staticDefenses)
            x1 = mapFeatures.staticDefenses(j, 1); y1 = mapFeatures.staticDefenses(j, 2);
            x2 = uasPoints(i, 1); y2 = uasPoints(i, 2);
            dist = sqrt((x2-x1)^2+(y2-y1)^2);
            if dist <= mapFeatures.staticDefenses(j, 3)
                SDHits(i, j) = 1;
            end
        end    
    end
end

% compare to mobile defenses
% MOBILE DEFENSE PLACEHOLDER
if height(mapFeatures.mobileDefenses>0)
    for i = 1:height(uasPoints)
        for j = 1:height(mapFeatures.mobileDefenses)
            x1 = mobileDefensePoints(i, 1); y1 = mobileDefensePoints(i, 2);
            x2 = uasPoints(i, 1); y2 = uasPoints(i, 2);
            dist = sqrt((x2-x1)^2+(y2-y1)^2);
            if dist <= mapFeatures.mobileDefenses(SMD, 3)
                MDHits(i, SMD) = 1;
            end    
        end
    end
end


end %EOF