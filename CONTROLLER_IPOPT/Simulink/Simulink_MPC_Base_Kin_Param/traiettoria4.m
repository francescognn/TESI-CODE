close all
clear 

%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.2;    %TIME STEP [s]
N = 15;      %N 

t0 = 0;     %START TIME [s]
tc = 2;     %ACCELERATION TIME [s]
tr = 10;     %RALLENTING TIME
trr = 15;    %REACCELERATING TIME
tf = 40;    %FINISH TIME [s]

delta_th=50;%DELTA ORIENTATION [deg]
dis=10;     %SPATIAL DISPLACEMENT [m]
Tsample=0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tt = 0:T:tf;
s=zeros(1,length(tt));
v=s; om =s;

vc = 0.8;
r=2;

vold = pi/4/(trr-tr)*r;
for i=1:length(tt)
    if tt(i)<=t0+tc
        v(i) = vc/tc*tt(i);
        om(i) = 0; 
    elseif tt(i) <= t0+tr
        v(i) = vc;
        om(i) = 0;       
    elseif tt(i) <= t0+trr
        om(i) = pi/4/(trr-tr);
        v(i) = r*om(i);
    elseif tt(i) <= t0+trr+6
        v(i) = v(i-1) + (vc-vold)/6*T;
        om(i) = -pi/4/6;
    elseif tt(i) <= tf-tc
        v(i) = vc;
        om(i) = 0;
    else 
        v(i) = vc/tc*(tf-tt(i));
        om(i) = 0;
    end
end
x = 0; y = 0; th = 0;
for i = 2:length(tt) 
    th(i) = th(i-1) + om(i-1)*T;
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end
th_d = om;
x_d = v.*cos(th);
y_d = v.*sin(th);
p_d = [x_d' y_d'];


figure;
plot(x,y,'*')
% axis([0 40 0 40])

Ttot=tf;

xd=[x;y+ones(size(y))*0;th];