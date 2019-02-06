close all
clear 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%   ARC TRAJECTORY   %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%
nometraj='traj_raffronto_NO_param_vinc';
T = 0.5;    %TIME STEP [s]
N = 15;      %N 

t0 = 0;     %START TIME [s]
tc = 10;     %ACCELERATION TIME [s]
tf = 20;    %FINISH TIME [s]

delta_th=30;%DELTA ORIENTATION [deg]
dis=10;     %SPATIAL DISPLACEMENT [m]
Tsample=0.01;

q0=[0 0 0 pi/4 -pi/4 pi/4 -pi/4 pi/4 pi/4]';
[Pee0,A]=jacobian_MM(q0);

x0_val = [q0;Pee0];

% p_0=zeros(7,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tt = 0:T:tf;
s=zeros(1,length(tt));
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

for i=1:length(tt)
    if tt(i)<=tc
        s(i) = s0 + vc/(2*tc)*tt(i)^2;
        v(i) = vc/tc*tt(i);
        dv(i) = vc/tc;
        
        th(i) = th0 + omc/(2*tc)*tt(i)^2;
        om(i) = omc/tc*tt(i);
        dom(i) = omc/tc;
    elseif tt(i) <= tf-tc
        s(i) = s0 + vc*(tt(i)-tc/2);
        v(i) = vc;
        dv(i) = 0;
        
        th(i) = th0 + omc*(tt(i)-tc/2);
        om(i) = omc;
        dom(i) = 0;
    else 
        s(i) = sf - vc/(2*tc)*(tf-tt(i))^2;
        v(i) = vc/tc*(tf-tt(i));
        dv(i) = -vc/tc;
        
        th(i) = thf - omc/(2*tc)*(tf-tt(i))^2;
        om(i) = omc/tc*(tf-tt(i));
        dom(i) = -omc/tc;
    end
end

% figure;
% subplot(3,1,1)
% plot(tt,s)
% title('s');
% subplot(3,1,2)
% plot(tt,v)
% title('v');
% subplot(3,1,3)
% plot(tt,dv)
% title('a');
% 
% figure;
% subplot(3,1,1)
% plot(tt,th)
% title('th');
% subplot(3,1,2)
% plot(tt,om)
% title('om');
% subplot(3,1,3)
% plot(tt,dom)
% title('om dot');

th_d = om;
x_d = v.*cos(th);
y_d = v.*sin(th);
p_d = [x_d' y_d'];

x = 0; y = 0;
for i = 2:length(tt) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

% figure;
% plot(x,y,'*')
% axis([0 30 0 30]);

Ttot=tf;

xd=[x;y;th;zeros(6,size(x,2));x;y;1.6.*ones(size(x));zeros(3,size(x,2))];

xb=x;
yb=y;
thb=th;
