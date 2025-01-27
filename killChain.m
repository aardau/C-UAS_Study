


%

function killData = killChain(adversaryPosition, mapFeatures)
    for j = 1:size(mapFeatures.staticDefenses,1) %FOR EACH STATIC DEFENSE
        for i = 1:size(adversaryPosition,2) %FOR EACH POINT IN THE UAS FLIGHT PATH

            %Calculate the distance from staticDefense #j to each x,y pair
            %of UAS flight path
            distanceToUAS = sqrt((mapFeatures.staticDefenses(j,1)-adversaryPosition(1,i))^2 + (mapFeatures.staticDefenses(j,2)-adversaryPosition(2,i))^2);
            
            %if the distance is less than effector #j's range, say its
            %detected.

            if distanceToUAS <= mapFeatures.staticDefenses(j,3)
                disp("UAS intersect")
                
            end
        end
    end
end

