function [pnew, U, unew, Xcomp,telapsed,man] = solver_NLP(xd_val,p0,x0,u0_val,sw)

import casadi.*

%% TIC
tstart=tic;

%% Recall the MXsym functions and variables to define the problem

Costfunction=evalin('base', 'Costfunction');
Gfunction=evalin('base', 'Gfunction');
all_samples=evalin('base','all_samples');
Usym=evalin('base','Usym');

p=evalin('base', 'p');
T_horizon=evalin('base', 'T_horizon');
N=evalin('base', 'N');

lbg=evalin('base', 'lbg');
ubg=evalin('base', 'ubg');

%% Defining the problem (i.e. giving xd,u0 and x0)

nlp    = struct('x', p, 'f', Costfunction(xd_val,p,x0,sw),'g', Gfunction(p,u0_val));
options = struct;
options.ipopt.tol=1e-3;
solver = nlpsol('solver','ipopt', nlp, options);

%% SOLVING PROBLEM (i.e. giving p0)

sol = solver('x0',p0,'lbg', lbg, 'ubg', ubg);

sol.x

pnew=full(sol.x);

%% COMPUTING U

unew = zeros(8,N);

%Nel caso volessi tutto u durante la predizione
% 
% for k=1:length(T_horizon)
% 
% unew(:,k) = full(Usym(pnew,T_horizon(k)).');
% 
% end
% U =  unew(:,1);

U = [pnew(4);pnew(8);pnew(9:end)];
 
%% Computing Xcomputed for the predicted Horizon 

 Xcomp=[full(all_samples(x0, repmat(pnew,1,N),T_horizon))];

 man=manipulability_index(Xcomp(1:9,1));
% Xcomp=zeros(size(16,N+1));

%% TOC
telapsed=toc(tstart);

end

