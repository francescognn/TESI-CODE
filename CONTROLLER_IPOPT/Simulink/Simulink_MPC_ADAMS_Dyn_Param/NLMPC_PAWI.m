function [u_new, U, pnew] = NLMPC_PAWI(N,T,xd,x0,p0)

% xd must be a 3 x N+1 matrix
% Tis the discretizing time of the given desired trajectory
% x0 is a 3 x 1 vector
% p0 is a 8 x 1 vector

import casadi.*

Ak=diag([5 5 0]);

fs = 1/T; % Sampling frequency [hz]
m  = 3;
T_horizon=(0:N-1)*T;

Xd=xd(:,1:N+1); %[linspace(0,2,N);2*ones(1,N);zeros(1,N)];

%%%%%%%%%%%% MODELING %%%%%%%%%%%%%%%%%%%%%
x  = MX.sym('x',3);
t  = MX.sym('t');

p = MX.sym('p',7);

xdot = [(p(1)*t^2+p(2)*t+p(3)*t)*cos(x(3)); ...
        (p(1)*t^2+p(2)*t+p(3)*t)*sin(x(3)); ...
        (p(4)*t^3+p(5)*t^2+p(6)*t+p(7))   ];

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
X_forecast = [x0,all_samples(x0, repmat(p,1,N),T_horizon)];

J=0;

for i=1:N

ex=(X_forecast(1,i)-Xd(1,i));
ey=(X_forecast(2,i)-Xd(2,i));
eth=(X_forecast(3,i)-Xd(3,i));
    
J = J + ((i/N)^m)*abs([ex ey eth]*Ak*[ex ey eth].');

%G = [G, {[(2*p(1).*T_horizon(i)+p(2))-0.4, (3*p(4).*T_horizon(i).^2+p(5).*T_horizon(i)+p(6))-0.5]}];
% devo scriverli come se fossero p<0 percui ogni elemento del vettore
% (lineari)
end

nlp = struct('x', p, 'f', J);
solver = nlpsol('solver','ipopt', nlp);

sol = solver('x0',p0);

pnew=full(sol.x);

U = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
      (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];
  
u_new = U(:,1);

Xcomp=[x0,full(all_samples(x0, repmat(pnew,1,N),T_horizon))];

end

