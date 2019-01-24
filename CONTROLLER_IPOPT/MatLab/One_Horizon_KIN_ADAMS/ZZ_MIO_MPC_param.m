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


Ak   = diag([10    10    1]);
A_ee = diag([1 1 1 1 1 1]);

%%% NB xd è valutato a partire dal passo 1, non dal passo 0 dell'orizzonte
%%% di predizione
P_cont_d_val=[xd(1:2,2:N+1); ones(1,N); zeros(3,N); xd(:,2:N+1); zeros(6,N)];%[linspace(0,2,N+1);2*ones(1,N+1);zeros(1,N+1)];
psi0=zeros(15,1);
p0=zeros(32,1);
u0_val=[0;0];
x0=[0 0.2 0 0 0.25 0 zeros(1,6)].';

%% MODELING 

P_cont_d  =  MX.sym('P_cont_d',15,N); % sono [x_ee y_ee z_ee th_ee phi_ee chi_ee x y th 0 0 0 0 0 0]'xN
t         =  MX.sym('t');
U0        =  MX.sym('U0',2,1);
TH1P   =  MX.sym('TH1P');
TH2P   =  MX.sym('TH2P');
TH3P   =  MX.sym('TH3P');
TH4P   =  MX.sym('TH4P');
TH5P   =  MX.sym('TH5P');
TH6P   =  MX.sym('TH6P');
TH1    =  MX.sym('TH1');
TH2    =  MX.sym('TH2');
TH3    =  MX.sym('TH3');
TH4    =  MX.sym('TH4');
TH5    =  MX.sym('TH5');
TH6    =  MX.sym('TH6');
x      =  MX.sym('x');
y      =  MX.sym('y');
th     =  MX.sym('th');



psi   =  [TH1P TH2P TH3P TH4P TH5P TH6P TH1 TH2 TH3 TH4 TH5 TH6 x y th ]';

p         =  MX.sym('p',4*8); % NEW CONTROL VARIABLE

% xdot = [(p(1)*t^2+p(2)*t+p(3))*cos(x(3)); ...
%         (p(1)*t^2+p(2)*t+p(3))*sin(x(3)); ...
%         (p(4)*t^3+p(5)*t^2+p(6)*t+p(7))         ];

%% Building control action 

Ff = [t^3  t^2  t   1];
Ffdot = [3*t^2 2*t 1 0];

Fb = [Ff 0 0 0 0; 0 0 0 0 Ff];
Fbdot=[Ffdot 0 0 0 0; 0 0 0 0 Ffdot];

Fm = [                         Ff      zeros(1,4*5)   ; ...
            zeros(1,4*1)       Ff      zeros(1,4*4)   ; ...
            zeros(1,4*2)       Ff      zeros(1,4*3)   ; ...
            zeros(1,4*3)       Ff      zeros(1,4*2)   ; ...
            zeros(1,4*4)       Ff      zeros(1,4*1)   ; ...
            zeros(1,4*5)       Ff                     ];

Fmdot = [                      Ffdot      zeros(1,4*5)   ; ...
            zeros(1,4*1)       Ffdot      zeros(1,4*4)   ; ...
            zeros(1,4*2)       Ffdot      zeros(1,4*3)   ; ...
            zeros(1,4*3)       Ffdot      zeros(1,4*2)   ; ...
            zeros(1,4*4)       Ffdot      zeros(1,4*1)   ; ...
            zeros(1,4*5)       Ffdot                     ];
        
u = [            Fm                  zeros(size(Fm,1),size(Fb,2))  ; ...
      zeros(size(Fb,1),size(Fm,2))                 Fb              ]*p; % [Tau1 Tau2 Tau3 Tau4 Tau5 Tau6 v omega]'

udot = [            Fmdot                  zeros(size(Fmdot,1),size(Fbdot,2))  ; ...
         zeros(size(Fbdot,1),size(Fmdot,2))                 Fbdot              ]*p; % [Tau1p Tau2p Tau3p Tau4p Tau5p Tau6p vp omegap]'
     
%% Recalling dynamic matrices and ode formulation
     
  [A, B, C] = dyn_matrices(TH1P, TH2P, TH3P, TH4P, TH5P, TH6P, TH1, TH2, TH3, TH4, TH5, TH6, x, y, th ,u, udot);

Amatrix = Function('Amatrix',{TH1P, TH2P, TH3P, TH4P, TH5P, TH6P, TH1, TH2, TH3, TH4, TH5, TH6, x, y, th},{A});
Bmatrix = Function('Bmatrix',{TH1P, TH2P, TH3P, TH4P, TH5P, TH6P, TH1, TH2, TH3, TH4, TH5, TH6, x, y, th},{B});
Cmatrix = Function('Cmatrix',{TH1P, TH2P, TH3P, TH4P, TH5P, TH6P, TH1, TH2, TH3, TH4, TH5, TH6, x, y, th, p},{C});

         
%%%%%%%///////////()()()()()()
        
% DEVO DEFINIRE LE MATRICI A E B, per il momento metto identità per vedere
% se il resto gira

psidot=A*psi+B*u+C; % anche C dipende da 

% Form an ode function
ode = Function('ode',{psi,p,t},{psidot});

%%  Creating a simulator 
N_steps_per_sample = 1;
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(psi,p,t);
k2 = ode(psi+dt/2.0*k1,p,t+dt/2.0);
k3 = ode(psi+dt/2.0*k2,p,t+dt/2.0);
k4 = ode(psi+dt*k3,p,t+dt/2.0);

psistep = psi+dt/6.0*(k1+2*k2+2*k3+k4);

%% Create a function that simulates one step propagation in a sample

one_step = Function('one_step',{psi, p, t},{psistep});

Psi = psi;

for i=1:N_steps_per_sample
    Psi = one_step(Psi, p, t);
end

%%%%%%%///////////()()()()()()

% Passo alla posizione dell'ee 

[Jp,Jo,T] = jacobians([Psi(13:15); Psi(7:12)]);

T11 = T{end}(1,1);
T12 = T{end}(1,2);
T13 = T{end}(1,3);
T21 = T{end}(2,1);
T22 = T{end}(2,2);
T23 = T{end}(2,3);
T31 = T{end}(3,1);
T32 = T{end}(3,2);
T33 = T{end}(3,3);

th_ee  = atan2(T23,T33);
c2     = sqrt(T11^2+T12);
phi_ee = atan2(-T13,c2);
chi_ee = atan2(sin(th_ee)*T31-cos(th_ee)*T21,cos(th_ee)*T22-sin(th_ee)*T32);

    
P_ee = [T{end}(1:3,4); th_ee; phi_ee; chi_ee; Psi(13:15); zeros(6,1)];




% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{psi, p, t}, {P_ee});

% speedup trick: expand into scalar operations
% one_sample = one_sample.expand();% NON FUNZIA NN SO PERCHE'

%%***TTTTTT>>>>>>

telapsed(2)=toc(tstart);

%% Simulating the system 

all_samples = one_sample.mapaccum('all_samples',N);


%%%%%%%%%%%% Identifying the simulated system: single shooting strategy %%%%%%%%%%

% Note, it is in general a good idea to scale your decision variables such
% that they are [0 0.2 0in the order of ~0.1..100
P_cont_forecast = [all_samples(psi0, repmat(p,1,N),T_horizon)];

%% GETTING EE POSITION

% usym_base = Fb*p(1:8);
% 
% usym_man  = Fm*p(9:end);
%     
% Usym=Function('Usym',{p,t},{usym});
  
%% COST FUNCTION DEFINITION

% DEVO RIPRENDERE DA QUI A METTERE A POSTO


J=0;

g={};
lbg=[];
ubg=[];

for i=1:N

    ex   = (P_cont_forecast(7,i)-P_cont_d(7,i));
    ey   = (P_cont_forecast(8,i)-P_cont_d(8,i));
    eth  = (P_cont_forecast(9,i)-P_cont_d(9,i));
    
    e_ee = (P_cont_forecast(1:3,i)-P_cont_d(1:3,i));

    J = J + ((i/N)^m)*abs([ex ey eth]*Ak*[ex ey eth].' + (e_ee.'*A_ee(1:3,1:3)*e_ee));

    %% CONSTRAINTS FUNCTION ( lbg<=g<=ubg )
    % 
        % if i==1
        %     
        % g = {g{:}, Usym(p,T_horizon(i))-U0};
        % 
        % else
        %     
        % g = {g{:}, Usym(p,T_horizon(i))-Usym(p,T_horizon(i-1))};
        % 
        % end
    % 
    % lbg = [lbg; [-Vpmax -Wpmax].'];
    % ubg = [ubg; [ Vpmax  Wpmax].'];
    % 
end

% Gfunction=Function('Gfunction',{p,U0}, {vertcat(g{:})});

Costfunction=Function('Costfunction',{P_cont_d,p}, {J});

%%***TTTTTT>>>>>>

telapsed(3)=toc(tstart);

%% NLP DEFINITION

nlp = struct('x', p, 'f', Costfunction(P_cont_d_val,p));%, 'g', Gfunction(p,u0_val));
solver = nlpsol('solver','ipopt', nlp);

%% SOLVING PROBLEM

sol = solver('x0',p0);%,'lbg', lbg, 'ubg', ubg);
% 
% sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
%             'lbg', lbg, 'ubg', ubg);

%%***TTTTTT>>>>>>

telapsed(4)=toc(tstart);

sol.x

pnew=full(sol.x);

%% COMPUTING U

u = [ (pnew(25).*T_horizon.^3+pnew(26).*T_horizon.^2+pnew(27).*T_horizon+pnew(28).*ones(size(T_horizon))); ...
      (pnew(29).*T_horizon.^3+pnew(30).*T_horizon.^2+pnew(31).*T_horizon+pnew(32).*ones(size(T_horizon)))];
 
% u = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
%       (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];

  
Xcomp=[zeros(15,1) full(all_samples(x0, repmat(pnew,1,N),T_horizon))];
Xcomp=Xcomp(:,1:end-1);

abs_xy_err=sqrt((Xcomp(7,:)-P_cont_d_val(7,:)).^2+(Xcomp(8,:)-P_cont_d_val(8,:)).^2);
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
title('errore assoluto norma xy')

figure

plot(Xcomp(7,:),Xcomp(8,:),'-o')
hold on 
grid on
plot(P_cont_d_val(7,:),P_cont_d_val(8,:),'-*')
legend('computed','desired')


eeee=abs(Xcomp-P_cont_d_val);

eeee=eeee(1:9,:);

figure

for i = 1:3

    plot(eeee(i,:))
    hold on
    drawnow
    
end
legend('err xee','err yee','err zee');%,'err th_1','err th_2','err th_3','err x','err y','err th')

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



