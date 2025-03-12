function [cost] = path_cost(path, path_aci, func, L, R, speed, cost_discretization)
    
    cost = 0;
    for i = 1:length(path_aci)-2
        points_aci = drawaction(path(i, :), L, R, path_aci(i+1), cost_discretization);
        cost_aci = integrate_cost(points_aci, speed, func);
        cost = cost + cost_aci;
    end
    
end

