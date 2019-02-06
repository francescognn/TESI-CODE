close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

traj_raffronto
clearvars -EXCEPT xb yb thb

nometraj='sin_complex';

q0=[xb(end) yb(end) 0  -1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
[Pee0,A]=jacobian_MM(q0);

p0=Pee0(1:3);

x0_val = [0;0;0;q0(4:end);Pee0];

N=15;
T = 0.5;
tf = 25;   
deltaz=0.15;
noscillazioni = 1.5;

Tsample=0.01;
dis=1.5; 
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



Ttot=tf;

xd1=[xb; yb;thb;[-1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]'*ones(1,size(xb,2)); [x(1);y(1);z(1)]*ones(1,size(xb,2)); zeros(3,size(xb,2))];

xd2=[[xb(end); yb(end);thb(end)]*ones(1,size(x,2));[-1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]'*ones(1,size(x,2));x;y;z;zeros(3,size(x,2))];

xd=[xd1,xd2];

figure;
plot3(xd(10,:),xd(11,:),xd(12,:),'*')
hold on
plot3(xd(1,:),xd(2,:),xd(3,:).*0)
grid on
axis([-1 12 -1 5 0 2]);
