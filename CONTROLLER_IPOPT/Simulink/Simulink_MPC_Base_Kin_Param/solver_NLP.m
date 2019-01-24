function [pnew, U, u, Xcomp,telapsed] = solver_NLP(xd_val,p0,x0,u0)


import casadi.*

tstart=tic;

Costfunction=evalin('base', 'Costfunction');
p=evalin('base', 'p');
T_horizon=evalin('base', 'T_horizon');
N=evalin('base', 'N');
all_samples=evalin('base','all_samples');
Gfunction=evalin('base', 'Gfunction');
lbg=evalin('base', 'lbg');
ubg=evalin('base', 'ubg');

nlp = struct('x', p, 'f', Costfunction(xd_val,p,x0), 'g', Gfunction(p,u0));
solver = nlpsol('solver','ipopt', nlp);


sol = solver('x0', p0,'lbg', lbg, 'ubg', ubg);


pnew=full(sol.x);

% u = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
%       (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];
%   

u = [ (pnew(1).*T_horizon.^3+pnew(2).*T_horizon.^2+pnew(3).*T_horizon+pnew(4).*ones(size(T_horizon))); ...
      (pnew(5).*T_horizon.^3+pnew(6).*T_horizon.^2+pnew(7).*T_horizon+pnew(8).*ones(size(T_horizon)))];
 
U = u(:,1);
  
Xcomp=[full(all_samples(x0, repmat(pnew,1,N),T_horizon))];

telapsed=toc(tstart);

end

