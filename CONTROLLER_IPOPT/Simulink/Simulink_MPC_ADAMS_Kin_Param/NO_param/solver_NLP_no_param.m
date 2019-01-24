function [ U, unew, Xcomp,telapsed,man] = solver_NLP_no_param(xd_val,x0,u0,sw)

import casadi.*

%% TIC
tstart=tic;

%% Recall the MXsym functions and variables to define the problem

Costfunction=evalin('base', 'Costfunction');
Gfunction=evalin('base', 'Gfunction');
all_samples=evalin('base','all_samples');

u=evalin('base', 'u');
T_horizon=evalin('base', 'T_horizon');
N=evalin('base', 'N');

lbg=evalin('base', 'lbg');
ubg=evalin('base', 'ubg');

%% Defining the problem (i.e. giving xd,u0 and x0)

nlp    = struct('x', u, 'f', Costfunction(xd_val,u,x0,sw),'g', Gfunction(u,u0(1:8,1)));
solver = nlpsol('solver','ipopt', nlp);

%% SOLVING PROBLEM (i.e. giving p0)

sol = solver('x0',u0,'lbg', lbg, 'ubg', ubg);

sol.x

unew=full(sol.x);

U = [unew(1:8,1)];
 
%% Computing Xcomputed for the predicted Horizon 

 Xcomp=[full(all_samples(x0, reshape(unew,8,N),T_horizon))];

 man=manipulability_index(Xcomp(1:9,1));
% Xcomp=zeros(size(16,N+1));

%% TOC
telapsed=toc(tstart);

end

