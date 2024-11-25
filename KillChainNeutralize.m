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
%% kinetic function
function Kinetickill = Kinetic_Neutralize(target_location, projectile_location, fireRate_refreshRate, in_range, tracking_target)
   %% target contact 
    if  target_location == projectile_location 
        Kinetickill = 1;
        target_location = [];
        projectile_location = []; 
        %taking both off the map, not accounting for debris
    else
        Kinetickill = 0;
        %projectile and target will continue on designated paths
    end
    weapon_range=weapon_location + [40,40]; % weapon location will be on the
                                            % map, so no specific location yet
                                            % added 40 units of range, not
                                            % sure how this will translate
                                            % to the map yet. 

    % have to write in about how fast the projectile is going 
    %% refresh fire rate
    fireRate_refreshRate = 10; %firerate timestep

    if fireRate_refreshRate > 0
        while true
            fireRate_refreshRate = fireRate_refreshRate - 1;
            fire_projectile = 0;
            if fireRate_refreshRate <= 0 && in_range==1
                fire_projectile = 1;
                fireRate_refreshRate = 10;
                break
            end
        end
    end
end
%% laser function (havent added any other DEWs)
function DEW_kill = DEW_Neutralize(target_location, laser_location, energy_AOE, time_to_kill, in_range)
    %speed of light, so time from fire->hit is negligable
    % create an AoE matrix to eliminate target if comes in range
    %energy does not need a reload time

%laser
    if DEW_kill==1
        target_location = [];
        laser_location = [];
    end
    laser_range=laser_location+[50,50]; % 50 range, not sure how this will
                                        % translate to the map yet
    time_to_kill=50; 
    % while locations are equal and in range, the laser will count down its 
    % time to kill. once target is killed, target will disappear (empty array), 
    % and the time to kill a target will reset. Loop will break, then must
    % track new target. 
    
    if  target_location == laser_location && in_range==1
        while true
            time_to_kill=time_to_kill-1;                
            if time_to_kill<=0
                DEW_kill=1;
                time_to_kill=50;
                break
            
            end
        end
    end
end
    
    

