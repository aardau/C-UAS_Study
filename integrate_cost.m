function [cost] = integrate_cost(candpath, speed, cost_func)
    
    N = size(candpath, 1);
    cost = 0;
    
    %center numerical integration
    for i = 1:N-1
        dc = (cost_func(candpath(i,1), candpath(i,2)) + cost_func(candpath(i+1,1), candpath(i+1,2)))/2;
        dt = norm(candpath(i, 1:2) - candpath(i+1, 1:2))/speed;
        cost = cost + dc*dt;
    end
    
end

