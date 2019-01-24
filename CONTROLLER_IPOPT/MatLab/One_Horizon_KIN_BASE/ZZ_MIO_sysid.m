%% INIZIALIZATION

clear all
close all
clc



tstart=tic;

import casadi.*

%%***TTTTTT>>>>>>

telapsed(1)=toc(tstart);

%% CHIAMO TRAIETTORIA DESIDERATA

ZZ_MIO_STEP

clearvars -EXCEPT xd N T tstart  telapsed
close all

%% SETTINGS
% N  = 10;  % Number of samples
% T  = 0.2;
fs = 1/T; % Sampling frequency [hz]
m  = 2;
T_horizon=(0:N-1)*T;


Vpmax=inf;
Wpmax=1.6;


Ak= diag([100    100    1]);


%%% NB xd è valutato a partire dal passo 1, non dal passo 0 dell'orizzonte
%%% di predizione
xd_val=xd(:,2:N+1);%[linspace(0,2,N+1);2*ones(1,N+1);zeros(1,N+1)];
x0=[0;0;0];
p0=zeros(8,1);
u0_val=[0;0];

%% MODELING 
x   = MX.sym('x',3);
Xd  = MX.sym('Xd',3,N);
t   = MX.sym('t');
U0  = MX.sym('U0',2,1);

p = MX.sym('p',8);

% xdot = [(p(1)*t^2+p(2)*t+p(3))*cos(x(3)); ...
%         (p(1)*t^2+p(2)*t+p(3))*sin(x(3)); ...
%         (p(4)*t^3+p(5)*t^2+p(6)*t+p(7))         ];

% A=MX(3,3);
% A(2,3)=(p(1)*t^3+p(2)*t^2+p(3)*t+p(4));

xdot = [(p(1)*t^3+p(2)*t^2+p(3)*t+p(4))*cos(x(3)); ...
        (p(1)*t^3+p(2)*t^2+p(3)*t+p(4))*sin(x(3)); ...
        (p(5)*t^3+p(6)*t^2+p(7)*t+p(8))         ];% + A*x ;

% Form an ode function
ode = Function('ode',{x,p,t},{xdot});

%%%%%%%%%%%% Creating a simulator %%%%%%%%%%
N_steps_per_sample = 1;
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(x,p,t);
k2 = ode(x+dt/2.0*k1,p,t+dt/2.0);
k3 = ode(x+dt/2.0*k2,p,t+dt/2.0);
k4 = ode(x+dt*k3,p,t+dt/2.0);

xstep = x+dt/6.0*(k1+2*k2+2*k3+k4);

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{x, p, t},{xstep});

X = x;
for i=1:N_steps_per_sample
    X = one_step(X, p, t);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{x, p, t}, {X});

% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%%***TTTTTT>>>>>>

telapsed(2)=toc(tstart);

%%%%%%%%%%%% Simulating the system %%%%%%%%%%

all_samples = one_sample.mapaccum('all_samples',N);

%%%%%%%%%%%% Identifying the simulated system: single shooting strategy %%%%%%%%%%

% Note, it is in general a good idea to scale your decision variables such
% that they are [0 0.2 0in the order of ~0.1..100
X_forecast = [all_samples(x0, repmat(p,1,N),T_horizon)];

usym = [p(1)*t^3+p(2)*t^2+p(3)*t+p(4); ...
        p(5)*t^3+p(6)*t^2+p(7)*t+p(8)];
    
Usym=Function('Usym',{p,t},{usym});
  
%% COST FUNCTION DEFINITION

J=0;

g={};
lbg=[];
ubg=[];

for i=1:N

ex=(X_forecast(1,i)-Xd(1,i));
ey=(X_forecast(2,i)-Xd(2,i));
eth=(X_forecast(3,i)-Xd(3,i));
    
J = J + ((i/N)^m)*abs([ex ey eth]*Ak*[ex ey eth].');

% CONSTRAINTS FUNCTION ( lbg<=g<=ubg )

if i==1
    
g = {g{:}, Usym(p,T_horizon(i))-U0};

else
    
g = {g{:}, Usym(p,T_horizon(i))-Usym(p,T_horizon(i-1))};

end

lbg = [lbg; [-Vpmax -Wpmax].'];
ubg = [ubg; [ Vpmax  Wpmax].'];

end

Gfunction=Function('Gfunction',{p,U0}, {vertcat(g{:})});

Costfunction=Function('Costfunction',{Xd,p}, {J});

%%***TTTTTT>>>>>>

telapsed(3)=toc(tstart);

%% NLP DEFINITION

%G = [(2*p(1).*T_horizon+p(2))-0.4, (3*p(4).*T_horizon.^2+p(5).*T_horizon+p(6))-0.5];

% Add inequality constraint


nlp = struct('x', p, 'f', Costfunction(xd_val,p), 'g', Gfunction(p,u0_val));
solver = nlpsol('solver','ipopt', nlp);

% solver.subject_to(2*p(1)*t+p(2)<=0.4 )
% solver.subject_to(3*p(4)*t^2+p(5)*t+p(6)<=0.5 )
% 

%% SOLVING PROBLEM

sol = solver('x0',p0,'lbg', lbg, 'ubg', ubg);
% 
% sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
%             'lbg', lbg, 'ubg', ubg);

%%***TTTTTT>>>>>>

telapsed(4)=toc(tstart);

sol.x

pnew=full(sol.x);

%% COMPUTING U

u = [ (pnew(1).*T_horizon.^3+pnew(2).*T_horizon.^2+pnew(3).*T_horizon+pnew(4).*ones(size(T_horizon))); ...
      (pnew(5).*T_horizon.^3+pnew(6).*T_horizon.^2+pnew(7).*T_horizon+pnew(8).*ones(size(T_horizon)))];
 
% u = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
%       (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];

  
Xcomp=[x0 full(all_samples(x0, repmat(pnew,1,N),T_horizon))];

% 
% for k=1:size(Xcomp1,2)   
%     Xcomp(:,k)=[cos(Xcomp1(3,k))  -sin(Xcomp1(3,k)) 0; ...
%                 sin(Xcomp1(3,k))   cos(Xcomp1(3,k)) 0; ...
%                 0                 0               1]*Xcomp1(:,k);
% end


xd_val=[xd(:,1) xd_val];

abs_xy_err=sqrt((Xcomp(1,:)-xd_val(1,:)).^2+(Xcomp(2,:)-xd_val(2,:)).^2);
max(abs_xy_err)

%% FINAL PLOTS

figure

plot(T_horizon,u(1:2,:),'-o')
hold on
grid on
legend('v','omega')

figure

plot(abs_xy_err)
grid on


figure

plot(Xcomp(1,:),Xcomp(2,:),'-o')
hold on 
grid on
plot(xd_val(1,:),xd_val(2,:),'-*')
legend('computed','desired')

%%***TTTTTT>>>>>>

telapsed(5)=toc(tstart);

figure

set(gca,'xticklabel', 1);
tdiff=diff(telapsed);
tesec=[telapsed(1) tdiff(1:end-1) telapsed(5)];

for k=1:5
plot(k-1,tesec(k),'s','MarkerSize',10,'LineWidth',20)
hold on
grid on
end
legend('importing time','definitions','propagazione stato','soluzione','whole script')

vp=diff(u(1,:));
wp=diff(u(2,:));

Vpmax_lev=Vpmax*ones(length(vp));
Wpmax_lev=Wpmax*ones(length(wp));

figure

plot(vp,'b')
hold on
grid on
plot(wp,'r')
plot(Vpmax_lev,'c')%,'linewidth',2)
plot(Wpmax_lev,'k')%,'linewidth',2)

legend('delta v','delta omega','V max','omega max');


% 
% h = bar(tesec.');
% colormap(summer(5));
% grid on
% 
% l = cell(1,length(tesec));
% l{1}='importing time'; l{2}='definitions'; l{3}='propagazione stato'; l{4}='soluzione'; l{5}='whole script';  
% legend(h,l);



