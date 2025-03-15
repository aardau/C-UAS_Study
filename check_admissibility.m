function [adm] = check_admissibility(obstacles, newnode, current, turnrad, aci)
    global domain;
    N = 2;

    %%%check solution domain admissibility
    admissdomain = true(size(current.node,1), 1);
    for i = 1:N
        in = newnode.node(:, i) >= domain(i,1) & newnode.node(:,i) <= domain(i,2);
        admissdomain = admissdomain & in;
    end
    adm = admissdomain;
    
    n = aci; %need to pass type of action being performed (S U D)
    obsnum = obstacles.number;
    switch n
        case 2 %if the action is turning up 
            u = [-sin(current.node(3)) cos(current.node(3))]*turnrad; %vector ccw perpendicular to current pose * turnrad
            p1 = current.node(1:2); 
            d1 = newnode.node(1:2)-p1;
            c = p1 + u; % center of the turn arc
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p2 = verts(j,:); %start and endpoint for edge
                    d2 = verts(j+1,:)-p2;
                    % checking for the intersection
                    delta = p2 - c;
                    cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - turnrad^2); %if roots are not complex the line and the circle intersect
                    if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                        t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                        t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                        if t1 >= 0 && t1 <= 1
                            p = p2 + t1*d2;
                            A = p-p1;
                            B = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end
                        if t2 >= 0 && t2 <= 1 
                            p = p2 + t2*d2;
                            A = p-p1;
                            B = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end 
                    end
                end
            end 
            
        case 3 %if the action is turning up
            u = [sin(current.node(3)) -cos(current.node(3))]*turnrad; %vector ccw perpendicular to current pose * turnrad
            p1 = current.node(1:2);
            d1 = newnode.node(1:2)-p1;
            c = p1 + u; % center of the turn arc
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p2 = verts(j,:); %start and endpoint for edge
                    d2 = verts(j+1,:)-p2;
                    % checking for an intersection
                    delta = p2 - c;
                    cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - turnrad^2); %if roots are not complex the line and the circle intersect
                    if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                        t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                        t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                        if t1 >= 0 && t1 <= 1
                            p = p2 + t1*d2;
                            B = p-p1; %because turn down is going cw need to swap bounds of the arc
                            A = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end
                        if t2 >= 0 && t2 <= 1 
                            p = p2 + t2*d2;
                            B = p-p1; %because turn down is going cw need to swap bounds of the arc
                            A = d1;
                            if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                                adm = 0;
                                return;
                            end
                        end 
                    end
                end
            end
            
        case 1 %if the action is going straight
            p0 = current.node(1:2); %start and end for path
            d0 = newnode.node(1:2)-p0;
            for i = 1:obsnum
                %for each obstacle check the path with each edge
                verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
                for j = 1:size(verts, 1)-1
                    p1 = verts(j,:); %start and endpoint for edge
                    d1 = verts(j+1,:)-p1;
                    %checking for an intersection
                    denom = d0(1)*d1(2) - d1(1)*d0(2); % the lines intersect if this product is not zero
                    if denom ~= 0
                        delt = p1 - p0;
                        s = (delt(1)*d1(2) - d1(1)*delt(2))/denom;
                        t = (delt(1)*d0(2) - d0(1)*delt(2))/denom;
                        %if the lines intersect andd the point is on both
                        %the obstacle line and the path return and set adm
                        %to false
                        if (s>=0 && s<=1) && (t>=0 && t<=1) 
                            adm = 0;
                            return;
                        end
                    end
                end
            end 
        otherwise 
            fprintf('Error, invalid case.\n')
    end
end

