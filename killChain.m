
%plan:
% Take x,y from all UAS and test their collision for each static defense.

function killData = killChain(x,y, mapFeatures)
    for j = 1:size(mapFeatures.staticDefenses,1) %FOR EACH STATIC DEFENSE
        for i = 1:length(x) %FOR EACH POINT IN THE UAS FLIGHT PATH
            distanceToUAS = sqrt((mapFeatures.staticDefenses(j,1)-x(i))^2 + (mapFeatures.staticDefenses(j,2)-y(i))^2);
    
            if distanceToUAS <= mapFeatures.staticDefenses(j,3)
                disp("UAS intersect")
            end
        end
    end
end

