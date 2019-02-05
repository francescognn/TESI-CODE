close all
clear 
%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

p0 = [-0.448,-0.685,1.36]; %INITIAL POSITION [x,y,z] 

N = 10;
T = 0.4;
tc = 6;     %ACCELERATION TIME [s]
tf = 15;    %FINISH TIME [s]
deltaz=0.3;

delta_th=30;%DELTA ORIENTATION [deg]
dis=1;     %SPATIAL DISPLACEMENT [m]
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
        z(i)= z0+deltaz/tc*tt(i);
    elseif tt(i) <= tf-tc
        v(i) = vc;
        om(i) = omc;
        z(i)= z(i-1)-deltaz/(tf-tc)*T;
    else 
        v(i) = vc/tc*(tf-tt(i));
        om(i) = omc/tc*(tf-tt(i));
        z(i)= z(i-1)-0.2/(tf-tc)*T;
    end
end

x = p0(1); y = p0(2); th=0;
for i = 2:length(tt) 
    th(i)= th(i-1)+om(i-1)*T;
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

figure;
plot3(x,y,z,'*')
grid on
axis([-1 1 -1 1 0 2]);

Ttot=tf;

xd=[x;y;z];