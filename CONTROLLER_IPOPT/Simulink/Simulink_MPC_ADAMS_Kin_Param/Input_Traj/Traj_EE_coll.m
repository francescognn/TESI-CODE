close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

nometraj='collision_kinect';

q0=[0 0 0  -deg2rad(169)  -deg2rad(164)  deg2rad(101)   -deg2rad(139)   deg2rad(270)   deg2rad(83)]';
[Pee0,A]=FK(q0);

p0=Pee0(1:3);

x0_val = [0;0;0;q0(4:end);Pee0;0;0;0];

N=25;
T = 0.2;
tc = 6;     %ACCELERATION TIME [s]
tf = 15;    %FINISH TIME [s]
deltaz=0.3;

delta_th=300;%DELTA ORIENTATION [deg]
dis=-0.3;     %SPATIAL DISPLACEMENT [m]
Tsample=0.01;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tt = 0:T:tf;
z=zeros(1,length(tt));
v=z; om =z;
r=(dis/2)/sin(deg2rad(delta_th/2));
l = r*deg2rad(delta_th);
s0 = 0; 
sf = l; 
vc = (sf-s0)/(tf-tc);
th0 = deg2rad(0);
thf = deg2rad(delta_th);
omc = (thf-th0)/(tf-tc);
z0=p0(3);
for i=1:length(tt)
    if tt(i)<=tc
        v(i) = vc/tc*tt(i);
        om(i) = omc/tc*tt(i);
        z(i)= z0;
    elseif tt(i) <= tf-tc
        v(i) = vc;
        om(i) = omc;
        z(i)= z0;
    else 
        v(i) = vc/tc*(tf-tt(i));
        om(i) = omc/tc*(tf-tt(i));
        z(i)= z0;
    end
end

x = p0(1); y = p0(2); th=0;
for i = 2:length(tt) 
    th(i)= th(i-1)+om(i-1)*T;
    x(i) = x(i-1) + v(i-1)*T*sin(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*cos(th(i-1));    
end

figure;
plot3(x,y,z,'*')
grid on

Ttot=tf;

xd=[zeros(9,size(x,2));x;y;z;zeros(3,size(x,2))];
