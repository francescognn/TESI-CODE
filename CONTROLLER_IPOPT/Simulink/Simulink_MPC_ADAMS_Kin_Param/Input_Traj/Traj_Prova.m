clear 
close
clc

T=0.15;
Tsample=0.01;
Ttot=20;
N=20;

%p_0=zeros(1,7);
% Npoint=7;
% 
% [xd,x0,tt] = traj_gen(T,Ttot,Npoint);
% 
% xd=xd.';

%% 
a=0.5;
b=0.2;
c=0.2;
d=0.19;

sec=1;
freq=1;

tt=0:T:Ttot;

xdx=a*tt;
xdy=(c.*(tt-sec).^3+d.*(tt-sec).^2-80)/400+b*sin(freq*(tt-sec)+pi/2);
xdy(1:round((sec/T)))=0.2;

xpdx=a*ones(1,length(tt));
xpdy=(3*c*(tt-sec).^2+2*d*(tt-sec))/400+b*freq*cos(freq*(tt-sec)+pi/2);
xpdy(1:round((sec/T)))=0;

xdth=atan2(xpdy,xpdx);

xd=[xdx; xdy; xdth]; %qui xd lo definisco in questa maniera in modo
                     %da poterlo gestire con il demux

xpd=zeros(3,length(tt));

xppd=zeros(3,length(tt));
