close all
clear 

%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.25;    %TIME STEP [s]
N = 20;      %N 

t0 = 0;     %START TIME [s]
tc = 2;     %ACCELERATION TIME [s]
tf = 30;    %FINISH TIME [s]

delta_th=50;%DELTA ORIENTATION [deg]
dis=10;     %SPATIAL DISPLACEMENT [m]
Tsample=0.01;


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
for i=(1+length(tt)):(2*length(tt))
    if tt(i)<=tf+tc
        s(i) = s0 + vc/(2*tc)*tt(i)^2;
        v(i) = vc/tc*tt(i);
        dv(i) = vc/tc;
        
        th(i) = th0 - omc/(2*tc)*tt(i)^2;
        om(i) = -omc/tc*tt(i);
        dom(i) = omc/tc;
    elseif tt(i) <= 2*tf-tc
        s(i) = s0 + vc*(tt(i)-tc/2);
        v(i) = vc;
        dv(i) = 0;
        
        th(i) = th0 -omc*(tt(i)-tc/2);
        om(i) = -omc;
        dom(i) = 0;
    else 
        s(i) = sf - vc/(2*tc)*(tf-tt(i))^2;
        v(i) = vc/tc*(tf-tt(i));
        dv(i) = -vc/tc;
        
        th(i) = thf + omc/(2*tc)*(tf-tt(i))^2;
        om(i) = -omc/tc*(tf-tt(i));
        dom(i) = omc/tc;
    end
end

th_d = om;
x_d = v.*cos(th);
y_d = v.*sin(th);
p_d = [x_d' y_d'];

x = 0; y = 0;
for i = 2:(2*length(tt)) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

% xflip=fliplr(x);
% yflip=fliplr(y);
% thflip = fliplr(th)
% fff = length(tt)
% for i = 1:length(tt)
%     x(fff+i)=x(fff)+xflip(i);
%     y(fff+i)=y(fff)-yflip(i);
%     th(fff+i)=th(fff)+xflip(i);
% end
figure;
plot(x,y,'*')


Ttot=tf;

xd=[x;y+ones(size(y))*0.3;th];