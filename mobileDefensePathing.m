function [defensePosition] = mobileDefensePathing(adversaryPosition, initialDefensePosition, mobileDefenseSpeed, dT)
%Path planning for mobile defenses
%   Currently, whatever mobile defense is first in the struct moves
%   linearly towards the current location of the UAS. Needs work to allow
%   'intellegent' targeting of which mobile defense goes and which UAS it
%   targets if there are multiple.

% create time vector
for i = 1:length(adversaryPosition)
    adversaryPosition(3, i) = 0+dT*(i-1);
end

defensePosition(:, 1) = initialDefensePosition;
for i = 1:(length(adversaryPosition)-1)
    ydif = adversaryPosition(2, i)-defensePosition(2, i);
    xdif = adversaryPosition(1, i)-defensePosition(1, i);
    theta = atan2(ydif, xdif);
    dx = mobileDefenseSpeed*cos(theta); dy = mobileDefenseSpeed*sin(theta);
    defensePosition(1, i+1) = defensePosition(1, i) + dx;
    defensePosition(2, i+1) = defensePosition(2, i) + dy;
end

end % EOF