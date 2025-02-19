function [defensePosition] = mobileDefensePathing(adversaryPosition, initialDefensePosition, mobileDefenseSpeed, dT)
%Path planning for mobile defenses
%   Currently, whatever mobile defense is first in the struct moves
%   linearly towards the current location of the UAS. Needs work to allow
%   'intellegent' targeting of which mobile defense goes and which UAS it
%   targets if there are multiple.

% create time vector
for i = 1:length(adversaryPosition)
    adversaryPosition(i, 3) = 0+dT*(i-1);
end

defensePosition(1, :) = initialDefensePosition;
for i = 1:(length(adversaryPosition)-1)
    ydif = adversaryPosition(i, 2)-defensePosition(i, 2);
    xdif = adversaryPosition(i, 1)-defensePosition(i, 1);
    theta = atan2(ydif, xdif);
    dx = mobileDefenseSpeed*cos(theta); dy = mobileDefenseSpeed*sin(theta);
    defensePosition(i+1, 1) = defensePosition(i, 1) + dx;
    defensePosition(i+1, 2) = defensePosition(i, 2) + dy;
end

end % EOF