%% INIZIALIZATION

% for kkk=1:5
%     for jjj=1:5


close all
clc

tstart=tic;

import casadi.*

%%***TTTTTT>>>>>>

telapsed(1)=toc(tstart);

%% CHIAMO TRAIETTORIA DESIDERATA

ZZ_MIO_STEP

clearvars -EXCEPT N T tstart  telapsed kkk jjj xd

close all

%% SETTINGS
% N  = 10;  % Number of samples
% T  = 0.2;
fs = 1/T; % Sampling frequency [hz]
m_b  = 20; %kkk
m_m  = 20; %jjj
T_horizon=(0:N-1)*T;

const_vec = [3 1.5 4 4 4 4 4 4].'; % = [Vpmax Wpmax TH1pmax TH2pmax TH3pmax TH4pmax TH5pmax TH6pmax] = [[m/s^2] [rad/s^2] [rad/s] [rad/s] ....]

const_vec = const_vec*T;


Ak_base = diag([10  10  5]);
Ak_ee   = diag([10 10 10 0.1 0.1 0.1]);

%%% NB xd è valutato a partire dal passo 1, non dal passo 0 dell'orizzonte
%%% di predizione

xd_val = [ xd(:,2:N+1); zeros(6,N); xd(1:2,2:N+1); 1.5*ones(1,N); zeros(3,N)] ;%[linspace(0,2,N+1);2*ones(1,N+1);zeros(1,N+1)];
xd_val0= [ xd(:,1); zeros(6,1); xd(1:2,1); 1.5*ones(1,1); zeros(3,1)] ;
p_ee0  = [ 0.0177    0.2544    0.4222  -0.1699   -2.0944    1.7407].';
x0     = [ zeros(3,1); pi/4*ones(6,1); p_ee0];
p0     = zeros(14,1);
u0_val = zeros(8,1);

%% MODELING 
x   = MX.sym('x',15); % [x y th TH1 TH2 TH3 TH4 TH5 TH6]
Xd  = MX.sym('Xd',15,N); % [x y th x_ee y_ee th_ee th1_ee th2_ee th3_ee]
t   = MX.sym('t');
U0  = MX.sym('U0',8,1);
p_ee = MX.sym('psi',6,1);

p = MX.sym('p',14);

Ff = [t^3  t^2  t   1];
Fp = [ 1 ];

Fb = [Ff 0 0 0 0; 0 0 0 0 Ff];

Lp=length(Fp);

Fm = [                          Fp      zeros(1,Lp*5)   ; ...
            zeros(1,Lp*1)       Fp      zeros(1,Lp*4)   ; ...
            zeros(1,Lp*2)       Fp      zeros(1,Lp*3)   ; ...
            zeros(1,Lp*3)       Fp      zeros(1,Lp*2)   ; ...
            zeros(1,Lp*4)       Fp      zeros(1,Lp*1)   ; ...
            zeros(1,Lp*5)       Fp                     ];
        
u = [            Fb                  zeros(size(Fb,1),size(Fm,2))  ; ...
      zeros(size(Fm,1),size(Fb,2))                 Fm              ]*p; % [v omega THp1 THp2 THp3 THp4 THp5 THp6]'
  

  
[P_ee,A] = jacobian_MM(x(1:9));    
    
xdot = A*u; % x dot è base + pos EE dot

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

xstep = [x+dt/6.0*(k1+2*k2+2*k3+k4)];
xstep = [ xstep(1:9);  P_ee];                      

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
X_forecast   = [all_samples(x0, repmat(p,1,N),T_horizon)];

Usym=Function('Usym',{p,t},{u});
  
%% COST FUNCTION DEFINITION

J=0;

g={};
lbg=[];
ubg=[];

for i=1:N
% 
% T=jacobian_MM_simple(X_forecast(:,i));

% [DHtable,A0] = parameters_MM_simple(X_forecast(:,i));
% 
% A = rototrasl(DHtable);
% 
% T=MX(4,4);
% T=A0;
% zeta=MX(3,10); 
% zeta(:,1) = A0(1:3,3);
% P=MX(3,10); 
% P(:,1) = A0(1:3,4);
% 
% for kk = 1:9
%     T = T*A{kk};
% end
% 
% X_forecast(4:6,i)=T(1:3,4);

ex      = (X_forecast(1,i)-Xd(1,i));
ey      = (X_forecast(2,i)-Xd(2,i));
eth     = (X_forecast(3,i)-Xd(3,i));

ex_ee   = (X_forecast(10,i)-Xd(10,i));
ey_ee   = (X_forecast(11,i)-Xd(11,i));
ez_ee   = (X_forecast(12,i)-Xd(12,i));
% eth1_ee = (X_forecast(7,i)-Xd(7,i));
% eth2_ee = (X_forecast(8,i)-Xd(8,i));
% eth3_ee = (X_forecast(9,i)-Xd(9,i));
% 

J = J + ((i/N)^m_b)*(  [ex ey eth]*Ak_base*[ex ey eth].'  )  +  ((i/N)^m_m)*(  [ex_ee ey_ee ez_ee]*Ak_ee(1:3,1:3)*[ex_ee ey_ee ez_ee].'  );

% CONSTRAINTS FUNCTION ( lbg<=g<=ubg )


if i==1
    
g = {g{:}, Usym(p,T_horizon(i))-U0};

elseif i==N
        
g = {g{:}, Usym(p,T_horizon(i))-Usym(p,T_horizon(i-1)), [ex_ee; ey_ee; ez_ee] };   

else
    
g = {g{:}, Usym(p,T_horizon(i))-Usym(p,T_horizon(i-1)) };

end

lbg = [lbg; -const_vec];
ubg = [ubg;  const_vec];

end
 
lbg = [lbg; zeros(3,1)]; % aggiungo terminal constraints
ubg = [ubg; zeros(3,1)];


Gfunction=Function('Gfunction',{p,U0,Xd}, {vertcat(g{:})});

Costfunction=Function('Costfunction',{Xd,p}, {J});

%%***TTTTTT>>>>>>

telapsed(3)=toc(tstart);

%% NLP DEFINITION

%G = [(2*p(1).*T_horizon+p(2))-0.4, (3*p(4).*T_horizon.^2+p(5).*T_horizon+p(6))-0.5];

% Add inequality constraint


nlp = struct('x', p, 'f', Costfunction(xd_val,p), 'g', Gfunction(p,u0_val,xd_val));
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

unew=zeros(8,N);

for k=1:length(T_horizon)

unew(:,k) = full(Usym(pnew,T_horizon(k)).');

end

% u = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
%       (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];

  

Xcomp = [x0 full(all_samples(x0, repmat(pnew,1,N),T_horizon))]; %full(one_sample(x0,pnew,T_horizon(1))) 
% 
% for k=1:N
% T=jacobian_MM_simple_val(Xcomp(:,k+1));
% Xcomp(4:6,k+1) = T(1:3,4);
% end

% T=jacobian_MM_simple_val(x0);
% Xcomp(4:6,1) = T(1:3,4);

 xd_val = [xd_val0(:,1) xd_val];
% 

abs_xy_err=sqrt((Xcomp(1,:)-xd_val(1,:)).^2+(Xcomp(2,:)-xd_val(2,:)).^2);
%max(abs_xy_err)

%% FINAL PLOTS

figure(1)

plot(T_horizon,unew(1:end,:),'-o')
hold on
grid on
legend('v','omega')

figure(2)

plot(abs_xy_err)
grid on
title('Abs error base xy')


figure(3)

plot(Xcomp(1,:),Xcomp(2,:),'-o')
hold on 
grid on
plot(xd_val(1,:),xd_val(2,:),'-*')
legend('computed','desired')

%%***TTTTTT>>>>>>

telapsed(5)=toc(tstart);


figure(4)
error_ee=sqrt((Xcomp(10,:)-xd_val(10,:)).^2+(Xcomp(11,:)-xd_val(11,:)).^2+(Xcomp(12,:)-xd_val(12,:)).^2);
plot(error_ee);
grid on
title('abs error of EE xyz')

% figure(5)
% 
% set(gca,'xticklabel', 1);
% tdiff=diff(telapsed);
% tesec=[telapsed(1) tdiff(1:end-1) telapsed(5)];
% 
% for k=1:5
% plot(k-1,tesec(k),'s','MarkerSize',10,'LineWidth',20)
% hold on
% grid on
% end
% legend('importing time','definitions','propagazione stato','soluzione','whole script')

figure(6)

plot3(Xcomp(10,:),Xcomp(11,:),Xcomp(12,:))

hold on
grid on

plot3(xd_val(10,:),xd_val(11,:),xd_val(12,:))

axis equal

legend('computed','desired')

%% PLOT DRAW

figure(7)

for j=1:size(Xcomp,2)
    [T,P] = TeP(Xcomp(1:9,j));
    plot3(P(1,3:4),P(2,3:4),P(3,3:4),'b','LineWidth',5)
    hold on
    plot3(P(1,4:end),P(2,4:end),P(3,4:end),'r','LineWidth',4)
    plot3(xd_val(10,:),xd_val(11,:),xd_val(12,:))
    grid on
    axis([[-2 3 -2 2 0 2]])
    hold off
    pause(0.06)
end

%% data out
% data=[m_b,m_m];
% 
% savefile = ['prova_mb-mm:' num2str(m_b)  '-'  num2str(m_m)  '.mat'];
% save(savefile,'data','Xcomp')
% 
%     end
% end


