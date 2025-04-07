function [updatedAdversaryPosition,killData] = killChain(adversaryPosition, mapFeatures)

    kill = false;
    
    for i = 1:size(adversaryPosition,2) %FOR EACH POINT IN THE UAS FLIGHT PATH
        for j = 1:size(mapFeatures.staticDefenses,1) %FOR EACH STATIC DEFENSE

            %Calculate the distance from staticDefense #j to each x,y pair
            %of UAS flight path
            distanceToUAS = sqrt((mapFeatures.staticDefenses(j,1)-adversaryPosition(1,i)')^2 + (mapFeatures.staticDefenses(j,2)-adversaryPosition(2,i)')^2);
            
            %if the distance is less than effector #j's range, say its
            %detected.

            if distanceToUAS <= mapFeatures.staticDefenses(j,3)
                disp("UAS intersect")
                kill = true;
            end
        end

        %Check for a kill at each point after iterating all defenses
        if kill == true 
            updatedAdversaryPosition = adversaryPosition(:,1:i);
            killData = adversaryPosition(:,i);
            return
        end
    end

    %if no kills awarded then return unchanged flight tracks and no kill
    if kill == false
        updatedAdversaryPosition = adversaryPosition;
        killData = [];
    end
end

 