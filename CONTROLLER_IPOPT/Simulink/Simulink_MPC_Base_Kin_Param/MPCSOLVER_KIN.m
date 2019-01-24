function [u_new, pnew] = MPCSOLVER_KIN(N, x0, p0, T, xs, xd)

%[u_new, U, pnew] = NLMPC_PAWI(N,T,xd,x0,p0);

%%%%%%%%%


load('WS.mat');

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

%G = [(2*p(1).*T_horizon+p(2))-0.4, (3*p(4).*T_horizon.^2+p(5).*T_horizon+p(6))-0.5];

nlp = struct('x', p, 'f', J);%, 'g', G);
solver = nlpsol('solver','ipopt', nlp);

pnew=full(sol.x);

U = [ (pnew(1).*T_horizon.^2+pnew(2).*T_horizon+pnew(3).*ones(size(T_horizon))); ...
      (pnew(4).*T_horizon.^3+pnew(5).*T_horizon.^2+pnew(6).*T_horizon+pnew(7).*ones(size(T_horizon)))];
  
u_new = U(:,1);

Xcomp=[x0,full(all_samples(x0, repmat(pnew,1,N),T_horizon))];



end
