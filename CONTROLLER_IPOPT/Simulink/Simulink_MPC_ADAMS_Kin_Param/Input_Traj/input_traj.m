function [N,T,Tsample,xd,nometraj,initialize_starting_point]=input_traj(N,T,Tsample,what,type,init)
% Function that returns the reference trajectory for our controller 
%      together with some parameters necessary for Simulink
%
%'what' is a string that can be: 'base' or 'MM'
%'type' is a string that can be: 
%   - 'line','curve' or 'L' for base motion
%   - 'line','sine' or 'NO' for MM motion

% RESTARTING FROM P0??
initialize_starting_point = init;

q0 = [0 0 0 pi/4 -pi/4 -pi/4 -pi/2 pi 0];
P0 = [0.3219;-0.2881;1.5766];

if isstring(type)==false
    error('the input trajectory type is not a string');
elseif isstring(what)==false
    error('choose between <<base>> and <<MM>> to control');
end

switch what
    case 'base'
        switch type 
            case 'line'
                t_total = 10;
                tt = 0:T:t_total;
                v = 0.5;
                xd = [v*tt;zeros(2,length(tt))];
                nometraj='Base_line';
            case 'curve'
                t_total = 20;
                radius = 3;
                thf = pi/2;
                tt = 0:T:t_total;
                th=linspace(0,thf,length(tt));
                xd=[radius*cos(th);-radius*sin(th)+radius;th];
                nometraj='Base_curve';
            case 'L'
                
                nometraj='Base_L';
            otherwise
                error('invalid trajectory type');
        end
    case 'MM'
        switch type 
            case 'line'
                
                nometraj='MM_line';
            case 'sine'
                
                nometraj='MM_sine';
            case 'NO'
                xd=[xx+P0(2);0.5.*ones(size(xx))+P0(2)/0.4;yy+1.2/0.4].*0.4;
                plot3(xd(1,:),xd(2,:),xd(3,:));
                xd=[zeros(9,size(xd,2));xd;-ones(1,size(xd,2));zeros(1,size(xd,2));zeros(1,size(xd,2));zeros(1,size(xd,2));ones(1,size(xd,2));zeros(1,size(xd,2))];

                nometraj='MM_NO';
            otherwise
                error('invalid trajectory type');
        end
    otherwise
        error('invalid trajectory type');
end


% q0=[0 0 0  1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
q0=[0 0 0  -deg2rad(177)  -deg2rad(73)  deg2rad(104)   -deg2rad(203)   deg2rad(263)   deg2rad(91)]';
[Pee0,A]=FK(q0);

p0=Pee0(1:3);


end



 
 
 