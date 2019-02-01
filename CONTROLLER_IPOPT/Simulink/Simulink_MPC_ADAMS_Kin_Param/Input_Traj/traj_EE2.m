close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

p0 = [-0.448,-0.685,1.36]; %INITIAL POSITION [x,y,z] 

T = 0.25;
tf = 15;   
deltaz=0.3;
noscillazioni = 2;

Tsample=0.01;
dis=1.2; 
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
axis([-1 1 -1 1 0 2]);

Ttot=tf;

xd=[x;y;z];