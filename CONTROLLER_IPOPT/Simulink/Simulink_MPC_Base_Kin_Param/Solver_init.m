

import casadi.*

% In this example, we fit a nonlinear model to measurements
%
% This example uses more advanced constructs than the vdp* examples:
% Since the number of control intervals is potentially very large here,
% we use memory-efficient Map and MapAccum, in combination with
% codegeneration.
%
% We will be working with a 2-norm objective:
% || y_measured - y_simulated ||_2^2
%
% This form is well-suited for the Gauss-Newton Hessian approximation.


%%%%%%%%%%% SETTINGS %%%%%%%%%%%%%%%%%%%%%

fs = 1/T; % Sampling frequency [hz]
m  = 1.1;
T_horizon=(0:N-1)*T;

Ak= diag([1000    1000    0.1]);



%[linspace(0,2,N+1);2*ones(1,N+1);zeros(1,N+1)];


%%%%%%%%%%%% MODELING %%%%%%%%%%%%%%%%%%%%%
x  = MX.sym('x',3);
t  = MX.sym('t');

p = MX.sym('p',7);

xdot = [(p(1)*t^2+p(2)*t+p(3))*cos(x(3)); ...
        (p(1)*t^2+p(2)*t+p(3))*sin(x(3)); ...
        (p(4)*t^3+p(5)*t^2+p(6)*t+p(7))         ];

% xdot = [(p(1)*t^3+p(2)*t^2+p(3)*t+p(4))*cos(x(3)); ...
%         (p(1)*t^3+p(2)*t^2+p(3)*t+p(4))*sin(x(3)); ...
%         (p(5)*t^3+p(6)*t^2+p(7)*t+p(8))         ];

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

save('WS.mat')
