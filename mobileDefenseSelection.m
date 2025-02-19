function [selectedMD] = mobileDefenseSelection(numMobileDefenses, MDArray, UASPath)
% The function decides which mobile defense is sent to intercept the
% incoming UAS threat. Based on initial positions of UAS breach and mobile
% defense
%   Detailed explanation goes here
if numMobileDefenses > 0
for n = 1:numMobileDefenses
    xmd = MDArray(n, 1);
    ymd = MDArray(n, 2);
    xuas = UASPath(1, 1); yuas = UASPath(1, 2);
    dist(n) = sqrt((xmd-xuas)^2+(ymd-yuas)^2);
end
end
[~, selectedMD] = min(dist)
dist
end