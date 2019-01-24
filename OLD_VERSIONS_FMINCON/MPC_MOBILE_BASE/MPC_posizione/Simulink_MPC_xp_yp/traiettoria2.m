close all
clear 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%   ARC TRAJECTORY   %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.7;    %TIME STEP [s]
N = 4;      %N 

t0 = 0;     %START TIME [s]
tc = 2;     %ACCELERATION TIME [s]
tf = 25;    %FINISH TIME [s]

delta_th=30;%DELTA ORIENTATION [deg]
dis=30;     %SPATIAL DISPLACEMENT [m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = 0:T:tf;
s=zeros(1,length(t));
v=s; dv=s;
th =s; om =s; dom = s;

r=(dis/2)/sin(deg2rad(delta_th/2));
% f=r*cos(deg2rad(delta_th/2));
l = r*deg2rad(delta_th);

s0 = 0; 
sf = l; 
vc = (sf-s0)/(tf-tc);

th0 = deg2rad(0);
thf = deg2rad(delta_th);
omc = (thf-th0)/(tf-tc);

for i=1:length(t)
    if t(i)<=tc
        s(i) = s0 + vc/(2*tc)*t(i)^2;
        v(i) = vc/tc*t(i);
        dv(i) = vc/tc;
        
        th(i) = th0 + omc/(2*tc)*t(i)^2;
        om(i) = omc/tc*t(i);
        dom(i) = omc/tc;
    elseif t(i) <= tf-tc
        s(i) = s0 + vc*(t(i)-tc/2);
        v(i) = vc;
        dv(i) = 0;
        
        th(i) = th0 + omc*(t(i)-tc/2);
        om(i) = omc;
        dom(i) = 0;
    else 
        s(i) = sf - vc/(2*tc)*(tf-t(i))^2;
        v(i) = vc/tc*(tf-t(i));
        dv(i) = -vc/tc;
        
        th(i) = thf - omc/(2*tc)*(tf-t(i))^2;
        om(i) = omc/tc*(tf-t(i));
        dom(i) = -omc/tc;
    end
end
% 
figure;
subplot(3,1,1)
plot(t,s)
title('s');
subplot(3,1,2)
plot(t,v)
title('v');
subplot(3,1,3)
plot(t,dv)
title('a');

figure;
subplot(3,1,1)
plot(t,th)
title('th');
subplot(3,1,2)
plot(t,om)
title('om');
subplot(3,1,3)
plot(t,dom)
title('om dot');

th_d = om;
x_d = v.*cos(th);
y_d = v.*sin(th);
p_d = [x_d' y_d'];

x = 0; y = 0;
for i = 2:length(t) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

% figure;
% plot(x,y,'*')
% axis([0 30 0 30]);

Ttot=tf;

xd=[x;y;th];