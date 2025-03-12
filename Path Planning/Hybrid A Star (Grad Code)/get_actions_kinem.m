function [actions] = get_actions_kinem(current, L, R, grid_x, grid_y, grid_th)
    %%%current pose
    Sx = current(1);
    Sy = current(2);
    Sth = current(3);

    %%%center of turn: turning "up"
    Cxup = Sx - R*sin(Sth);
    Cyup = Sy + R*cos(Sth);
    Cxdn = Sx + R*sin(Sth);
    Cydn = Sy - R*cos(Sth);

    %%%turn angles
    turnangle = L/R;
    newangle_up = Sth + turnangle;
    newangle_down = Sth - turnangle;

    %%%movements: new locations: 1) forward, 2) turn "up", 3) turn "down"
    actions.newlocation = [Sx + L*cos(Sth), Sy + L*sin(Sth), Sth;
        Cxup + R*sin(newangle_up), Cyup - R*cos(newangle_up), mod(newangle_up, 2*pi);
        Cxdn + R*sin(-newangle_down), Cydn + R*cos(-newangle_down), mod(newangle_down, 2*pi)];    

    actions.newindex = zeros(3,3);
    LX = length(grid_x);
    LY = length(grid_y);
    
    for i = 1:3
        location = actions.newlocation(i,:);
        
        xdiff = abs(grid_x - location(1));
        [~, xpmin] = min(xdiff);
        if xpmin == 1 &&  (grid_x(1) >  location(1))
            actions.newindex(i,1) = -inf;    %driven off the domain
        elseif xpmin == LX &&  (grid_x(LX) <  location(1))
            actions.newindex(i,1) = +inf;  %driven off the domain
        else
            actions.newindex(i,1) = xpmin;
        end
        
        ydiff = abs(grid_y - location(2));
        [~, ypmin] = min(ydiff);
        if ypmin == 1 &&  (grid_y(1) >  location(2))
            actions.newindex(i,2) = -inf;    %driven off the domain
        elseif ypmin == LY &&  (grid_y(LY) <  location(2))
            actions.newindex(i,2) = inf;  %driven off the domain
        else
            actions.newindex(i,2) = ypmin;        
        end
        
        %%%the th-index is cyclic. It will never drive off the domain
        thdiff = abs(grid_th - location(3));
        [~, thpmin] = min(thdiff);
        actions.newindex(i,3) = thpmin;
    end

    %%%obtain cumulative index
    for i = 1:3
        actions.newindexc(i) = (actions.newindex(i,3) - 1)*LX*LY + (actions.newindex(i,2) - 1)*LX + actions.newindex(i,1);
    end

end

