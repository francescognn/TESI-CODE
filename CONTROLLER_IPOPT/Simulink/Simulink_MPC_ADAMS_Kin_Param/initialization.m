
clearvars -EXCEPT  xd x0_val Pee0 N T Tsample tt nometraj

import casadi.*

%% PARAMETERS OF THE PROBLEM

m_b       = 3; 
m_m       = m_b;
m_j       = m_m;
Ak_base   = diag([100  100  0]);
Ak_joints = diag([100 100 100 100 100 100]);
Ak_ee     = diag([10^6 10^6 10^6 0.1 0.1 0.1]);
Ak_mm     = 0.001;
Ak_ub     = diag([1e5 1e7]);


const_vec = [  3    1.5    2        2       2       2       2       2 ].'; 
%         = [Vpmax Wpmax TH1pmax TH2pmax TH3pmax TH4pmax TH5pmax TH6pmax] 
%          [m/s^2][rad/s^2][rad/s][rad/s] [rad/s][rad/s] [rad/s] [rad/s]  
const_vec = const_vec*T;

%%% NB xd è valutato a partire dal passo 1, non dal passo 0 dell'orizzonte
%%% di predizione

fs = 1/T; % Sampling frequency [hz]
T_horizon=(0:N-1)*T;

%% MODELING 
x   = MX.sym('x',15); % [x y th TH1 TH2 TH3 TH4 TH5 TH6 manipulability_index]
p   = MX.sym('p',14); % [p1 p2 p3 p4 ... p14]
Xd  = MX.sym('Xd',15,N); % [x y th TH1 TH2 TH3 TH4 TH5 TH6 x_ee y_ee th_ee th1_ee th2_ee th3_ee]
t   = MX.sym('t');
U0  = MX.sym('U0',8,1); % [V W THP1 THP2 THP3 THP4 THP5 THP6]
x0  = MX.sym('x0',15);
p0  = MX.sym('p0',14);
sw  = MX.sym('sw',1); 

%%  Definisco il vettore u 

Ff = [t^3  t^2  t   1];
Fp = [ 1 ];

Fb = [Ff 0 0 0 0; 0 0 0 0 Ff];

Lp = length(Fp);

Fm = [                          Fp      zeros(1,Lp*5)   ; ...
            zeros(1,Lp*1)       Fp      zeros(1,Lp*4)   ; ...
            zeros(1,Lp*2)       Fp      zeros(1,Lp*3)   ; ...
            zeros(1,Lp*3)       Fp      zeros(1,Lp*2)   ; ...
            zeros(1,Lp*4)       Fp      zeros(1,Lp*1)   ; ...
            zeros(1,Lp*5)       Fp                     ];
        
u = [            Fb                  zeros(size(Fb,1),size(Fm,2))  ; ...
      zeros(size(Fm,1),size(Fb,2))                 Fm              ]*p; % [v omega THp1 THp2 THp3 THp4 THp5 THp6]'
  
%% Writing differential equation
 
[P_ee,Psi_ee,Rot_T] = FK(x(1:9));    
% G=[cos(x(3)) 0; sin(x(3)) 0; 0 1];
G=zeros(3,2);
xdot = [G zeros(3,6); zeros(6,2) eye(6); zeros(6,8)]*u;
% 
% [P_ee,A]=jacobian_MM(x(1:9))
% 
% xdot=A*u;


% Form an ode function
ode = Function('ode',{x,p,t},{xdot});

%%%%%%%%%%%% Creating a simulator %%%%%%%%%%
N_steps_per_sample = 1;
dt = 1/fs/N_steps_per_sample;

%%%%%%%%%%%% Building integrator %%%%%%%%%%%%
k1 = ode(x,p,t);
k2 = ode(x+dt/2.0*k1,p,t+dt/2.0);
k3 = ode(x+dt/2.0*k2,p,t+dt/2.0);
k4 = ode(x+dt*k3,p,t+dt/2.0);

xstep = [x+dt/6.0*(k1+2*k2+2*k3+k4)];

%%%%%%%%%%%% Adding EE pose calculation %%%%%%%%%%%%

xstep = [ xstep(1:9); P_ee; [0 0 0]'];                      

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{x, p, t},{xstep});

X = x;
for i=1:N_steps_per_sample
    X = [one_step(X, p, t)];
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{x, p, t}, {X});

% speedup trick: expand into scalar operations
  one_sample = one_sample.expand();

%% Simulating the system 

all_samples  = one_sample.mapaccum('all_samples',N);

X_forecast   = [all_samples(x0, repmat(p,1,N),T_horizon)];

Usym=Function('Usym',{p,t},{u}); % mi serve per calcolare le u una volta che ho le p alla fine dell'ottimizzazione
  
%% COST FUNCTION DEFINITION

J=0;

g={};
lbg=[];
ubg=[];

for i=1:N
    
ex       = (X_forecast(1,i)-Xd(1,i));
ey       = (X_forecast(2,i)-Xd(2,i));
eth      = (X_forecast(3,i)-Xd(3,i));

e_th1    = (X_forecast(4,i)-Xd(4,i));
e_th2    = (X_forecast(5,i)-Xd(5,i));
e_th3    = (X_forecast(6,i)-Xd(6,i));
e_th4    = (X_forecast(7,i)-Xd(7,i));
e_th5    = (X_forecast(8,i)-Xd(8,i));
e_th6    = (X_forecast(9,i)-Xd(9,i));

ex_ee    = (X_forecast(10,i)-Xd(10,i));
ey_ee    = (X_forecast(11,i)-Xd(11,i));
ez_ee    = (X_forecast(12,i)-Xd(12,i));
% eth1_ee = (X_forecast(13,i)-Xd(13,i));
% eth2_ee = (X_forecast(14,i)-Xd(14,i));
% eth3_ee = (X_forecast(15,i)-Xd(15,i));
man_i    =  manipulability_index(X_forecast(1:9,i));

u        = Usym(p,T_horizon(i));

J = J + ((i/N)^m_b)*(  [ex ey eth]*Ak_base*[ex ey eth].'  )*(1-sw)    + ...
        ((i/N)^m_b)*(  [e_th1 e_th2 e_th3 e_th4 e_th5 e_th6]*Ak_joints*[e_th1 e_th2 e_th3 e_th4 e_th5 e_th6].'  )*(1-sw) + ...
        ((i/N)^m_b)*(  [ex_ee ey_ee ez_ee]*Ak_ee(1:3,1:3)*[ex_ee ey_ee ez_ee].'  )*sw+ ...
        ((i/N)^m_b)*(  Ak_mm/(man_i).^2  )*sw+ ...
        ((i/N)^m_b)*(  u(1:2).'*Ak_ub*u(1:2))*sw;
       

% CONSTRAINTS FUNCTION ( lbg<=g<=ubg )

 [scav1,scav2]=SCA(X_forecast(1:9,i));

if i==1
    
g = {g{:}, [u-U0; X_forecast(4,i);scav1;scav2; X_forecast(5,i); X_forecast(6,i); X_forecast(7,i); X_forecast(8,i); X_forecast(9,i)]};

else

 g = {g{:}, [u-Usym(p,T_horizon(i-1));scav1;scav2; X_forecast(4,i); X_forecast(5,i); X_forecast(6,i); X_forecast(7,i); X_forecast(8,i); X_forecast(9,i)]};


end
    
lbg = [lbg; -const_vec; 0;  0; -deg2rad(350); -deg2rad(180); -deg2rad(145); -deg2rad(180); -deg2rad(350); -deg2rad(350)];
ubg = [ubg;  const_vec; inf; inf; deg2rad(350); deg2rad(0); deg2rad(145); deg2rad(0); deg2rad(350); deg2rad(350) ];

end


%% CREATING FUNCTION FOR J AND G
 
Gfunction=Function('Gfunction',{p,U0,x0}, {vertcat(g{:})});

Costfunction=Function('Costfunction',{Xd,p,x0,sw}, {J});

Gfunction=Gfunction.expand();

Costfunction=Costfunction.expand();

Usym=Usym.expand();
