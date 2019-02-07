 close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

nometraj='sin_';

% q0=[0 0 0  1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
q0=[0 0 0  -deg2rad(169)  -deg2rad(164)  deg2rad(101)   -deg2rad(139)   deg2rad(270)   deg2rad(83)]';
[Pee0,A]=FK(q0);

p0=Pee0(1:3);

x0_val = [q0;Pee0;0;0;0];

N=10;
T = 0.4;
tf = 25;   
deltaz=0.15;
noscillazioni = 1.5;

Tsample=0.01;
dis=0.5; 
v=dis/tf;
om=(noscillazioni*2*pi)/tf;
tt = 0:T:tf;
z=zeros(1,length(tt));
x=z; y=z;

x(1)=p0(1);y(1)=p0(2);
z=p0(3)+(deltaz/2)-(deltaz/2)*cos(om*tt);
for i=2:length(tt)
    x(i)=x(i-1)+v*T;
    y(i)=y(i-1)+v*T;
end

figure;
plot3(x,y,z,'*')
grid on
axis equal
%axis([-1 1 -1 1 0 2]);

Ttot=tf;

xd=[zeros(9,size(x,2));x;y;z;zeros(3,size(x,2))];