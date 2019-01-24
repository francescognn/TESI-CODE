function [xd_ee, xdp_ee] = trajectory_ee(T,Ttot)

t = 0:T:Ttot;
x = linspace(0.2,-0.2,length(t));
y = linspace(-0.2,0.2,length(t));
z = 1.3 + linspace(-0.1,0.1,length(t));

xp = zeros(1,length(t));
yp = zeros(1,length(t));
zp = zeros(1,length(t));

xd_ee = [x;y;z].';
xdp_ee = [xp;yp;zp].';

