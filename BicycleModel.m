%Richard Kelly
%Bicycle Equation of Motion for 2D UAV modelling
%Experimental Projects I, Capstone Project

clc;clear;close all force;

%fixedPoint
fixedPoint = [17500,17500];

%Constants
%max and min turn radius
turnRadius = 40; %deg
iterations = 50000;

%itialize variables to loop on.
x = zeros(1,iterations); %m
y = zeros(1,iterations); %m
theta = zeros(1,iterations); %deg
delta = zeros(1,iterations); %deg
v = zeros(1,iterations); %m/s
phi = zeros(1,iterations); %deg/s
xDistanceToFP = zeros(1,iterations); %m
yDistanceToFP = zeros(1,iterations); %m
alphaAngleToFP = zeros(1,iterations); %deg

%dT is the resolution of the simulation
dT = 0.1; %second

%% Initial Parameters
%starting position
x(1) = 0; %m
y(1) = 0; %m

%starting angle of the drone wrt to x axis -> "heading"
theta(1) = 70; %deg

%starting angle of the "front wheel" wrt to theta
delta(1) = 0; %deg

%fixed velocity
v(:) = 22 %m/s

%fixed steering rate
phi(:) = -0.0002 %deg/s


%% For Loop
for i = 1:iterations
   
    %Distance,Angle to FP
    xDistanceToFP(i) = fixedPoint(1) - x(i);
    yDistanceToFP(i) = fixedPoint(1) - y(i);
    alphaAngleToFP(i) = atand(yDistanceToFP(i)/xDistanceToFP(i));

    %Basic Tracking Routine V1:
    %First calculate the difference between "heading" theta and alpha
    angleDif = theta(i) - alphaAngleToFP(i)
    



    %Equations of Motion
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

plot(x,y,LineWidth=2,Color="r")
grid on
hold on
plot(fixedPoint(1),fixedPoint(2),color="b",Marker="square")
title("Bicycle Equations of Motion relative to a Fixed Point")
xlabel("x [m]")
ylabel("y [m]")