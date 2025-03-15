function actionpath = drawaction(current, L, R, actionum, cost_discretization)

    %%%in this one: there are only three possible actions: move straight by a
    %%%fixed length, left or right by the same length at the vehicles min turn
    %%%radius

    %%%current pose
    Sx = current(1);
    Sy = current(2);
    Sth = current(3);

    actionpath = current;
    turnangle = L/R;
    disclen = cost_discretization;   %number of intermediate points for plotting
    %%%center of turn: turning "up"
    if actionum == 0    %do nothing
        actionpath = repmat(current, disclen, 1);
    elseif actionum == 1    %move straight
        lendisc = linspace(0,L,disclen);
        for i = 1:disclen
            actionpath(1+i, :) = [Sx + lendisc(i)*cos(Sth), Sy + lendisc(i)*sin(Sth), Sth];
        end        
    elseif actionum == 2    %turn "up"
        Cxup = Sx - R*sin(Sth);
        Cyup = Sy + R*cos(Sth);
        newangle_up = Sth + turnangle;
        angledisc = linspace(Sth, newangle_up, disclen);
        for i = 1:disclen
            actionpath(1+i,:) = [Cxup + R*sin(angledisc(i)), Cyup - R*cos(angledisc(i)), mod(angledisc(i), 2*pi)];
        end    
    else   %turn down
        Cxdn = Sx + R*sin(Sth);
        Cydn = Sy - R*cos(Sth);
        newangle_down = Sth - turnangle;
        angledisc = linspace(Sth, newangle_down, disclen);
        for i = 1:disclen
            actionpath(1+i,:) = [Cxdn + R*sin(-angledisc(i)), Cydn + R*cos(-angledisc(i)), mod(angledisc(i), 2*pi)];
        end    
    end
end