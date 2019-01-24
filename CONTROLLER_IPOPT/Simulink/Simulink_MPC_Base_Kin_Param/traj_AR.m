close all
clear 


%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.2;    %TIME STEP [s]
N = 25;      %N 

t0 = 0;     %START TIME [s]
tf = 35;    %FINISH TIME [s]
r = 1;    %CURVATURE RADIUS

dis = 10;    %SPATIAL DISPLACEMENT [m]
vel = 0.5;    %LINEAR VELOCITY [m/s]
wturn = 0.2;%TURN TWIST [rad/s]
Tsample=0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_0=zeros(8,1);


tt = t0:T:tf;
s=zeros(1,length(tt)); v=s;th=s;w=s;
s0 = 0;
th0 = 0;

t1 = dis/vel;
t2 = dis/vel + pi/wturn;
t3 = 2*dis/vel + pi/wturn;
t4 = 2*dis/vel + 2*pi/wturn;
vturn = wturn*r;
in = length(t0:T:t4);
for i=1:length(tt)
    if tt(i) <= t1
        s(i) = s0 + vel*(tt(i)-t0);
        v(i) = vel;
                
        th(i) = th0; 
        w(i) = 0;        
    elseif tt(i)  <= t2
        s(i) = s0 + vel*(t1-t0) + vturn*(tt(i)-t1);
        v(i) = vturn;
        
        th(i) = th0 + wturn*(tt(i)-t1);
        w(i) = wturn;
    elseif tt(i)  <= t3
        s(i) = s0 + vel*(t1-t0) + vturn*(t2-t1) + vel*(tt(i)-t2);
        v(i) = vel;
        
        th(i) = pi;
        w(i) = 0;
    elseif tt(i)  <= t4
        s(i) = s0 + vel*(t1-t0) + vturn*(t2-t1) + vel*(t3-t2) + vturn*(tt(i)-t3);
        v(i) = vturn;
        
        th(i) = -pi + wturn*(tt(i)-t3);
        w(i) = wturn;
    else
        s(i) = s(i-in);
        v(i) = v(i-in);
        th(i) = th(i-in);
        w(i) = w(i-in);
    end
end

x = 0; y = 0;
for i = 2:length(tt) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

xd=[x;y;th];

figure;
plot(tt,s,tt,v,tt,th,tt,w);
legend('s','v','th','w')
grid on


figure;
plot(tt,[x ; y]);
grid on
legend('x','y');

figure;
for j = 1 :length(tt)
    plot(x(j),y(j),'*r');
    drawnow;
    hold on
    axis equal
end