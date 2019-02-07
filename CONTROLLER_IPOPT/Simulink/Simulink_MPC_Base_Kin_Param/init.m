clearvars -EXCEPT  xd N T Tsample tt disturb

import casadi.*


%%%%%%%%%%%% SETTINGS %%%%%%%%%%%%%%%%%%%%%

fs = 1/T; % Sampling frequency [hz]
m  = 2;
T_horizon=(0:N-1)*T;

Ak= diag([10    10    10]);

Vpmax=1.5;  %  [m/s^2]
Wpmax=4; %  [rad/s^2]

Vpmax=Vpmax*T;
Wpmax=Wpmax*T;
%%%%%%%%%%%% MODELING %%%%%%%%%%%%%%%%%%%%%
x  = MX.sym('x',3);
Xd  = MX.sym('Xd',3,N);
t  = MX.sym('t');
x0 = MX.sym('x0',3);
U0  = MX.sym('U0',2,1);

p = MX.sym('p',12);

xdot = [(p(1)*t^5+p(2)*t^4+p(3)*t^3+p(4)*t^2+p(5)*t+p(6))*cos(x(3)); ...
        (p(1)*t^5+p(2)*t^4+p(3)*t^3+p(4)*t^2+p(5)*t+p(6))*sin(x(3)); ...
        (p(7)*t^5+p(8)*t^4+p(9)*t^3+p(10)*t^2+p(11)*t+p(12))    ];

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


%%%%%%%%%%%% Simulating the system %%%%%%%%%%

all_samples = one_sample.mapaccum('all_samples', N);

%%%%%%%%%%%% Identifying the simulated system: single shooting strategy %%%%%%%%%%

% Note, it is in general a good idea to scale your decision variables such
% that they are in the order of ~0.1..100
X_forecast = [all_samples(x0, repmat(p,1,N),T_horizon)];

usym = [p(1)*t^5+p(2)*t^4+p(3)*t^3+p(4)*t^2+p(5)*t+p(6); ...
        p(7)*t^5+p(8)*t^4+p(9)*t^3+p(10)*t^2+p(11)*t+p(12)];
    
Usym=Function('Usym',{p,t},{usym});

J=0;
g={};
lbg=[];
ubg=[];

for i=1:N

ex=(X_forecast(1,i)-Xd(1,i));
ey=(X_forecast(2,i)-Xd(2,i));
eth=(X_forecast(3,i)-Xd(3,i));
    
J = J + ((i/N)^m)*abs([ex ey eth]*Ak*[ex ey eth].');

if i==1
    
g = {g{:}, Usym(p,T_horizon(i))-U0 };

else
    
g = {g{:}, Usym(p,T_horizon(i))-Usym(p,T_horizon(i-1)) };

end

lbg = [lbg; [-Vpmax -Wpmax].'];
ubg = [ubg; [ Vpmax  Wpmax].'];

end

Gfunction=Function('Gfunction',{p,U0}, {vertcat(g{:})});

Costfunction=Function('Costfunction',{Xd,p,x0}, {J});

% nlp = struct('x', p, 'f', Costfunction(xd_val,p));%, 'g', G);
% solver = nlpsol('solver','ipopt', nlp);
% 
% 
% sol = solver('x0',p0);

