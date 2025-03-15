function [cost] = cost_to_goal(start, goal, speed, costfunc)
    n = ceil(norm(start - goal)) + 1;
    dt = linspace(0, 1, n);
    s = (goal' + dt.*(start' - goal'))';
    
    cost = integrate_cost(s, speed, costfunc);
end
