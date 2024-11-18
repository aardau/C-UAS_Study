%Neglect the time it takes for projectile or energy to travel to target

% time accounting for weapon misses/error
% 
%% timing
% if projectile hits its target, the target will no longer be focused and
% the target will fall for a time
    %if projectile misses, fire again instantaneously

% DEWs time to kill with high power microwaves or lasers (AOE vs Pinpoint
% accuracy)(Fry circuits vs melt through)
    % how long does energy need to be focused on target to neutralize?
        % Laser: 10-60 seconds depending on how far away, how strong laser
        % is, and how strong material is
        %Microwaves: 1-10 seconds to fry circuits, depending on the same.
    % range of weapons and how much Area of Effect
    % if DEW runs out of power (especially for mobile effector), then it is
    % disabled until further notice
            % charge up time?
    %continuous energy output until target is neutralized

%% kill

% if projectile location = target location, then target is killed
    % when target is killed, remove target from simulation
%% code 
%kinetic (getting rid of projectile and target on collision)

%if fireRate_refreshTime=0;
    %weaponFire=1
%else 
    %weaponFire=0;
%end

%target_location=[,]; (defined location on a path)
%projectile_location=[,]; (defined location on a path)
%if projectile_location=target_location
    %projectile_location=[]; (empty brackets)
    % target location=[]; (empty brackets)
% else
    %target continues on path
    %projectile continues on path
%end
function Kinetic_kill = Kinetic_Neutralize(target_location, projectile_location, fireRate_refreshRate)
   %% target contact 
    if target_location == projectile_location 
        Kinetic_kill=1;
        target_location = [];
        projectile_location = []; %taking both off the map, not accounting for debris
    else
        Kinetic_kill = 0;
        %projectile and target will continue on designated paths
    end
    %% refresh fire rate
    fireRate_refreshRate = 10; %firerate timestep
    if fireRate_refreshRate > 0
        while true
            fireRate_refreshRate = fireRate_refreshRate - 1;
            fire_projectile = 0;
            if fireRate_refreshRate <= 0 
                fire_projectile = 1;
                fireRate_refreshRate = 10;
                break
            end
        end
    end
end

function DEW_kill = DEW_Neutralize(target_location, energy_location, energy_refreshRate, time_to_kill)
end
    
    

