close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%



q0=[0 0 0  -1.1352   -1.3124    1.4284   -1.9527    4.4346   -0.0100]';
[Pee0,A]=jacobian_MM(q0);

p0=Pee0(1:3);

N=10;
T = 0.25;
tf = 25;   
deltaz=0.3;
noscillazioni = 2;

Tsample=0.01;
dis=0.7; 
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
%axis([-1 1 -1 1 0 2]);

Ttot=tf;

xd=[zeros(9,size(x,2));x;y;z;zeros(3,size(x,2))];