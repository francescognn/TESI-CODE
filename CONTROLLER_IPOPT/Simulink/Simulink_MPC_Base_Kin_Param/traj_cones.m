close all 
clear all

%%%%%%%%%%%%  DATA  %%%%%%%%%%%%%

T = 0.25;    %TIME STEP [s]
N = 25;      %N 
disturb=0;  %disturbance 

t0 = 0;     %START TIME [s]
tc = 3;     %tempo per fare una curva
Ncurve=4;  %numero di curve
r=0.6;      %raggio della curva
t_acc=6;
Tsample=0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = tc*Ncurve+t_acc;
tt = t0:T:tf;
omc = pi/tc;
vc = omc*r;

v=zeros(1,length(tt)); om=v;
ncurva=1;
for i=1:length(tt)
    if tt(i)<=t_acc
        v(i) = vc/t_acc*tt(i);
        om(i) = 0;
    elseif mod(ncurva,2)==1
        v(i)=vc;
        om(i)=-omc;
    else
        v(i)=vc;
        om(i)=omc;
    end
    if mod(tt(i)-t_acc,tc)==0
        ncurva=ncurva+1;
    end
end

x = 0; y = 0; th = 0;
for i = 2:length(tt) 
    x(i) = x(i-1) + v(i-1)*T*cos(th(i-1));
    y(i) = y(i-1) + v(i-1)*T*sin(th(i-1));  
    th(i)=th(i-1) + om(i-1)*T;
end

x_cone = vc*(t_acc-t0)/2 * ones(Ncurve,1);
y_cone = (r:(2*r):(2*r*Ncurve))';
% figure;
% plot(x,y,'*')
% grid on 
% axis equal
% hold on
% % viscircles([x_cone y_cone],0.2*ones(Ncurve,1),'Color','r')
% plot(x_cone,y_cone,'.','MarkerSize',20)

Ttot=tf;

xd=[x;y;th];