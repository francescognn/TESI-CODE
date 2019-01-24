close all
clear 


%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.1;    %TIME STEP [s]
N = 6;      %N 

t0 = 0;     %START TIME [s]
tf = 30;    %FINISH TIME [s]
r = 1;    %CURVATURE RADIUS

dis = 3;    %SPATIAL DISPLACEMENT [m]
vel = 0.5;    %LINEAR VELOCITY [m/s]
wturn = 0.6;%TURN TWIST [rad/s]
Tsample=0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


t = t0:T:tf;
s=zeros(1,length(t)); v=s;th=s;w=s;
s0 = 0;
th0 = 0;

t1 = dis/vel;
t2 = dis/vel + pi/wturn;
t3 = 2*dis/vel + pi/wturn;
t4 = 2*dis/vel + 2*pi/wturn;
vturn = wturn*r;
in = length(t0:T:t4);
for i=1:length(t)
    if t(i) <= t1
        s(i) = s0 + vel*(t(i)-t0);
        v(i) = vel;
                
        th(i) = th0; 
        w(i) = 0;        
    elseif t(i)  <= t2
        s(i) = s0 + vel*(t1-t0) + vturn*(t(i)-t1);
        v(i) = vturn;
        
        th(i) = th0 + wturn*(t(i)-t1);
        w(i) = wturn;
    elseif t(i)  <= t3
        s(i) = s0 + vel*(t1-t0) + vturn*(t2-t1) + vel*(t(i)-t2);
        v(i) = vel;
        
        th(i) = pi;
        w(i) = 0;
    elseif t(i)  <= t4
        s(i) = s0 + vel*(t1-t0) + vturn*(t2-t1) + vel*(t3-t2) + vturn*(t(i)-t3);
        v(i) = vturn;
        
        th(i) = -pi + wturn*(t(i)-t3);
        w(i) = wturn;
    else
        s(i) = s(i-in);
        v(i) = v(i-in);
        th(i) = th(i-in);
        w(i) = w(i-in);
    end
end

x = 0; y = 0;
for i = 2:length(t) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));    
end

xd=[x;y;th];

% figure;
% plot(t,s,t,v,t,th,t,w);
% legend('s','v','th','w')
% grid on


% figure;
% plot(t,[x ; y]);
% grid on
% legend('x','y');
% 
% figure;
% for j = 1 :length(t)
%     plot(x(j),y(j),'*r');
%     drawnow;
%     hold on
%     axis equal
% end