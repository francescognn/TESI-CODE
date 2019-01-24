clear all
close all
clc

nometraj='PICK_UP_no_param';
T=0.5;
N=15;
Tsample=0.01;
Tf=20;

q0=[0 0 0 pi/4 pi pi/4 pi/4 pi/4 pi/4]';
[Pee0,A]=jacobian_MM(q0);

x0_val = [q0;Pee0];

T_drive=0.25;
%% Base Traj

% [xwp,ywp]=ginput(9);
% WP=[zeros(1,2);[xwp,ywp]*10];
% waypoints = WP;
waypoints = ...
[        0         0;
    1.2783    0.0683;
    2.6357    0.0956;
    4.6267    0.5328;
    5.3507    1.6530;
    5.4186    2.5546;
    5.4864    3.8115;
    6.0973    4.9044;
    8.0656    5.4508;
    9.2421    5.5328];
    
speeds = [0 1.2 1 0.8 0.5 0.5 0.8 1 1.2 0];

s = drivingScenario('SampleTime',T_drive,'StopTime',Tf);
 
    % construct a circular road
%     road(s, [40 10; 40 12]);
 
    % create a car that is 3 meters in length.
    v = vehicle(s,'Length',3);
  
    % drive the car at varying points and speeds along the circle.
    
    trajectory(v, waypoints, speeds)

    i=1;
    xd=[];
    while advance(s)
            xd(1,i)= v.Position(1);
            xd(2,i)= v.Position(2);
        i=i+1;
    end 
      
   xdp=(diff(xd.')./T).';
   
   th=[ 0 atan2(xdp(1,:),xdp(2,:))];
   
   xd=[xd;th];
   
   
   xd(4:9,:)=zeros(6,size(xd,2));

    clearvars -EXCEPT T N Tsample Tf nometraj xd speeds x0_val T_drive
    
%% Traj EE

s = drivingScenario('SampleTime',T_drive,'StopTime',Tf);

waypoints = ...
 [      0         0;
    1.3235    0.0410;
    3.2466    0.1230;
    4.5814    0.3415;
    6.2104    1.6530;
    7.2285    2.7186;
    6.8665    3.7842;
    6.4367    4.7951;
    7.3643    6.1066;
    8.7670    6.5710];

    % construct a circular road
%     road(s, [40 10; 40 12]);
 
    % create a car that is 3 meters in length.
    v = vehicle(s,'Length',3);
  
    % drive the car at varying points and speeds along the circle.
    
    trajectory(v, waypoints, speeds)

    i=1;
    while advance(s)
            xd(10,i)= v.Position(1);
            xd(11,i)= v.Position(2);
        i=i+1;
    end 
    
%% Z EE 
 clearvars -EXCEPT T N Tsample Tf nometraj xd speeds x0_val T_drive

 dz = 1.4;
 
waypoints = ...
[       0        dz;
    0.5769    0.1776+dz;
    1.5498    0.1322+dz;
    2.4548    0.1049+dz;
    3.3145    0.1689+dz;
    4.2647    0.1328+dz;
    4.9434    0.1514+dz;
    5.9842    0.1694+dz;
    6.8439    0.1962+dz;
    7.9072    0.1776+dz];

s = drivingScenario('SampleTime',T_drive,'StopTime',Tf);

    % create a car that is 3 meters in length.
    v = vehicle(s,'Length',3);
  
    % drive the car at varying points and speeds along the circle.
    
    trajectory(v, waypoints, speeds)

    i=1;
    while advance(s)
            xd(12,i)= v.Position(2);
             
        i=i+1;
    end 
for i=1:size(xd,2)
    if xd(12,i)==0
    xd(12,i)=0.1776+dz;
    end
    if xd(1,i)==0
    xd(1,i)=9.2421;
    end
    if xd(2,i)==0
    xd(2,i)=5.5328;
    end
end


  xd(13:15,:)=zeros(3,size(xd,2));
  xd(4:9,:)=[pi/4 pi pi/4 pi/4 pi/4 pi/4]'*ones(1,size(xd(4:9,:),2));
  