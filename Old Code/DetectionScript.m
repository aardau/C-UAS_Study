clc
clear

%Set paramaters for fixed and mobile effectors
    %Include field of view and range
    %Assumed that mobile is worse than fixed

%Create statement for detection
    %Pull adversary flight path
    %If the object crosses into the field of view and range, it is detected
    %If not, it is not detected
    %Need to set it up so that each effector is active and can detect an
    %object
    %varies based on number and type of effectors

%Outputs: p
    %Positive detection notification
    %Which effector detected the object to pass along to tracking
