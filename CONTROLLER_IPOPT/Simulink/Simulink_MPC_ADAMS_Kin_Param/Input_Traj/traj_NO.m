 close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

nometraj='sin_';

% q0=[0 0 0  1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
q0=[0 0 0  -deg2rad(177)  -deg2rad(73)  deg2rad(104)   -deg2rad(203)   deg2rad(263)   deg2rad(91)]';
[Pee0,A]=FK(q0);

p0=Pee0(1:3);

x0_val = [q0;Pee0;zeros(6,1)];

N=10;
T = 0.4;
Tsample=0.01;

 %%
%  
%  [xp,yp]=ginput(25);
%  grid on
 %%
%  t=1:25;
%  
%  tt=linspace(t(1),t(end),200);
%  
%  xx=spline(t,xp',tt);
%  yy=spline(t,yp',tt);
%  
 xd=[xx+Pee0(2);0.5.*ones(size(xx))+Pee0(2)/0.4;yy+1.2/0.4].*0.4;
 
 plot3(xd(1,:),xd(2,:),xd(3,:));
 
 xd=[zeros(9,size(xd,2));xd;-ones(1,size(xd,2));zeros(1,size(xd,2));zeros(1,size(xd,2));zeros(1,size(xd,2));ones(1,size(xd,2));zeros(1,size(xd,2))];
 
 
 