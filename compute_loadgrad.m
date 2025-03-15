function [loadgrad] = compute_loadgrad(back_parent_node, back_aci, L, R, speed, loadfunc)
    cost_discretization = 4;

    switch back_aci
        case 1
            up = integrate_cost(drawaction(back_parent_node, L, R, 2, cost_discretization), speed, loadfunc);
            dwn = integrate_cost(drawaction(back_parent_node, L, R, 3, cost_discretization), speed, loadfunc);     
            loadgrad = (up + dwn)/2;
        case 2
            str = integrate_cost(drawaction(back_parent_node, L, R, 1, cost_discretization), speed, loadfunc);
            up = integrate_cost(drawaction(back_parent_node, L, R, 2, cost_discretization), speed, loadfunc);
            dwn = integrate_cost(drawaction(back_parent_node, L, R, 3, cost_discretization), speed, loadfunc);
            loadgrad = (-3*up + 4*str - dwn)/2;
        case 3
            str = integrate_cost(drawaction(back_parent_node, L, R, 1, cost_discretization), speed, loadfunc);
            up = integrate_cost(drawaction(back_parent_node, L, R, 2, cost_discretization), speed, loadfunc);
            dwn = integrate_cost(drawaction(back_parent_node, L, R, 3, cost_discretization), speed, loadfunc);
            loadgrad = (3*dwn - 4*str + up)/2;
        otherwise
            loadgrad = 0;
    end
    loadgrad = abs(loadgrad);
end

