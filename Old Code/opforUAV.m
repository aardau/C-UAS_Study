%Richard Kelly
%Experimental Projects 1
%Function for spawning enemy UAV and producing x,y data for a flight track

function [x,y,totalDistanceToFP] = opforUAV(uavParameters,fixedPoint,pidTune,simParameters)
%% Purpose
% The opforUAV function returns the x,y data of a 2D top down flight track
% intended for use in a larger simulation spawning multiple UAVs flying 
% towards a fixed point at fpx,fpy. 

%% Expected inputs and returns
%expects UAVparameters as
% [initx,inity,heading,velocity,maxTurnRadius]

%expects fixedPoint as
% [fpx,fpy,endRadius]

%expects PID tune as
% [Kp, Ki, Kd]

%expects simParameters as
% [dT, iterations]

%returns [x,y] as positional data in the same length as iterations
%ISSUE: if the UAV overshoots the fixed point, x,y becomes non-sensical

%% Sim Parameters
dT = simParameters(1);
iterations = simParameters(2);

%% Pre-Allocation of Looping Variables
x = zeros(1,iterations); %m
y = zeros(1,iterations); %m
theta = zeros(1,iterations); %deg
delta = zeros(1,iterations); %deg
v = zeros(1,iterations); %m/s
phi = zeros(1,iterations); %deg/s
xDistanceToFP = zeros(1,iterations); %m
yDistanceToFP = zeros(1,iterations); %m
alphaAngleToFP = zeros(1,iterations); %deg
angleDif = zeros(1,iterations); %deg
totalDistanceToFP = zeros(1,iterations);

%% Initial Vehicle Parameters
%starting position
x(1) = uavParameters(1); %m
y(1) = uavParameters(2); %m

%starting angle of the drone wrt to x axis -> "heading"
theta(1) = uavParameters(3); %deg

%starting angle of the "front wheel" wrt to theta is already set as 0

%fixed velocity
v(:) = uavParameters(4); %m/s

%turn radius
turnRadius = uavParameters(5);

%% Applied PID Values
%these are set as zero prior to the loop
error_prior = 0; 
integral_prior = 0;

Kp = pidTune(1);   %Proportional Gain for PID controller
Ki = pidTune(2);       %Integral Gain for PID controller
Kd = pidTune(3);      %Derivatie Gain for PID controller

%% For Loop
    for i = 1:iterations
       
        %% Distance,Angle to FP
        xDistanceToFP(i) = fixedPoint(1) - x(i);
        yDistanceToFP(i) = fixedPoint(2) - y(i);
        alphaAngleToFP(i) = atan2d(yDistanceToFP(i),xDistanceToFP(i));
        totalDistanceToFP(i) = sqrt(xDistanceToFP(i)^2+yDistanceToFP(i)^2);
        if totalDistanceToFP(i) < fixedPoint(3)
            x(i+1:end) = [];
            y(i+1:end) = [];
            break
        end
        %% Basic Tracking Routine V1:
        %First calculate the difference between "heading" theta and alpha
        angleDif(i) = alphaAngleToFP(i) - theta(i);
     
        %PID controller terms
        integral = integral_prior + angleDif(i)*dT;
        derivative = (angleDif(i)-error_prior)/dT;
        
        %controller:
        phi(i) =  Kp*angleDif(i) + Ki*integral + Kd*derivative;
        
        %i-1 terms
        error_prior = angleDif(i);
        integral_prior = integral;
    
        %% Equations of Motion
        xDot = v(i) * cosd(delta(i) + theta(i));
        yDot = v(i) * sind(delta(i) + theta(i));
        thetaDot = v(i)*sind(delta(i));
        deltaDot = phi(i);
    
        %Add to next
        x(i+1) = x(i) + xDot.*dT;
        y(i+1) = y(i) + yDot.*dT;
        theta(i+1) = theta(i) + thetaDot.*dT;
        delta(i+1) = delta(i) + deltaDot.*dT;
    
        %Turn Radius Limiting
        if delta(i+1) > turnRadius
            delta(i+1) = turnRadius;
        elseif delta(i+1) < -turnRadius
            delta(i+1) = -turnRadius;
        end
      
    end 

end