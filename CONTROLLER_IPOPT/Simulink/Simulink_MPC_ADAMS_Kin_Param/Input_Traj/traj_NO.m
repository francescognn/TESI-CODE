 close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

nometraj='traj_NO';

% q0=[0 0 0  1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
q0=[0 0 0  -0.3271   -1.6893    1.4493   -0.1655    1.2337    1.1716]';
[Pee0,A, B]=FK(q0);

p0=Pee0(1:3);

x0_val = [q0;Pee0;zeros(6,1)];

N=10;
T = 0.4;
Tsample=0.01;

 %%
%  
%  [xp,yp]=ginput(25);
xp=[0;1;1.5;1.25;0.15];
yp=[0;-0.4;-1.17;-2;-2.5];

%  grid on

  t=1:5;
%  
 tt=linspace(t(1),t(end),91);
 
 xx=spline(t,xp',tt);
 yy=spline(t,yp',tt);

 dxx=diff(xx);
 dyy=diff(yy);
 th=atan2(dyy,dxx);
 xx=xx(1:end-1);
 yy=yy(1:end-1);
 

% load('Data_saved/datax_y.mat')

% 
% 
% sc=0.5;
% 
% xx=xx.*0.6;
% yy=yy.*0.6;
% 
%  xd=[xx+Pee0(1)-xx(1);Pee0(2).*ones(size(xx));yy+Pee0(3)-yy(1)];
%  
%  plot3(xd(1,:),xd(2,:),xd(3,:));
%  grid on
%  
%  xd=[zeros(9,size(xd,2));xd;zeros(1,size(xd,2));zeros(1,size(xd,2));ones(1,size(xd,2));zeros(1,size(xd,2));-ones(1,size(xd,2));zeros(1,size(xd,2))];

 