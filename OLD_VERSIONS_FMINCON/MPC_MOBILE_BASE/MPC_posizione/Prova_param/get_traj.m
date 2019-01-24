function [tt,xd,xpd,xppd] = get_traj(dt,T)

% genera una traiettoria un po' a caso dandogli il dt e il tempo totale
% da in uscita [tt xd xpd xppd]


ax=0.5;
bx=0.2;
cx=0.5;
dx=1;

ay=0.2;
by=0.19;
cy=0.2;
dy=4;

sec=1;
freq=1;

tt=0:dt:T;

xdx=ax*tt;
xdy=(ay.*(tt-sec).^3+by.*(tt-sec).^2-80)/400+bx*sin(freq*(tt-sec)+pi/2);
xdy(1:round(sec/dt))=0;

xpdx=ax;
xpdy=(3*ay*(tt-sec).^2+2*by*(tt-sec))/400+bx*freq*cos(freq*(tt-sec)+pi/2);
xpdy(1:round(sec/dt))=0;

xdth=atan2(xpdy,xpdx);

xd=[xdx.' xdy.' xdth.'];

xpd=[];

xppd=[];

end

