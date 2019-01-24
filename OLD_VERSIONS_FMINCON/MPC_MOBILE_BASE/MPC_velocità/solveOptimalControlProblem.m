
function [du, V] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system, N, t0, x0, dx0, du0, T, xs, xd, options)

% DESCRIZIONE DI solveOptimalControlProblem
% la funzione prende in ingresso: 
% 
% > runningunningcosts: è la parte della funzione di costo che va valutata per 
% ogni passo da 1 a N e poi sommata
% 
% cost = runningcosts(...)
% 
% > terminalcosts: parte della funzione di costo per i costi terminali (fuori dalla sommatoria
% 
% cost = terminalcosts(...)
% 
% > constraints: usata per vincoli su dx e du, 
% 
% [c ceq] = constraints(...)
% 
% > terminalconstraints: va usata per i vincoli finali (per il passo N) 
%   
% [c ceq] = terminalconstraints(...)
% 
% linearconstraints: usata per i vincoli lineari al solutore, 
% lavora sulla variabile da mibimizzare
% 
% [A, b, Aeq, beq, lb, ub] = linearconstraints(...)
%     
% > system: è una funzione che calcola come propagare lo stato (x e dx):
% 
% [y,dy] = system(...)
%     
% > N: orizzonte dell'MPC
% 
% > t0: istante iniziale
% 
% > x0: stato iniziale 
% 
% > dx0: variazione di stato iniziale
% 
% > du0: variazione di controllo stato iniziale 
% 
% > T: periodo tra un passo e un altro 
% 
% > xs: è lo stato desiderato modificato per tenere in considerazione l'obtacle avoidance
% 
% > xd: stato desiderato (senza ostacoli imprevisti)
% 
% > options: sono le opzioni del solutore fmincon

%%

   A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    
    [x,dx] = computeOpenloopSolution(system, N, T, t0, x0, dx0, du0);

    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),du0(:,k));
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end
    
[du V] = fmincon(@(du) costfunction(runningcosts, terminalcosts, system, ... 
     N, T, t0, x0, dx0, du, xs, xd), du0, A, b, Aeq, beq, lb, ub, @(du) ...
     constraintseval(constraints,terminalconstraints, system, N, T, t0, x0, dx0, du));
    

function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, T, t0, x0, dx0, du, xs, xd)
cost = 0;
    x  = zeros(N+1, length(dx0));
    dx = zeros(N+1, length(dx0));
    [x,dx] = computeOpenloopSolution(system, N, T, t0, x0, dx0, du);
    
    for k=1:N
        cost = cost+runningcosts(t0+k*T, dx(k,:), du(:,k), x(k,:), xs(k,:), xd(k,:));
    end
    cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:), dx(N+1,:),xs(N+1,:));

end

function [x,dx] = computeOpenloopSolution(system, N, T, t0, x0, dx0, du)
    x(1,:) = x0;
    dx(1,:)= dx0;
    for k=1:N
        [x(k+1,:),dx(k+1,:)] = system(t0+T*k,x(k,:), dx(k,:), du(:,k),T);
    end

end


function [c,ceq] = constraintseval ...
        (constraints, terminalconstraints, system, N, T, t0, x0, dx0, du)
    
    x  = zeros(N+1, length(dx0));
    dx = zeros(N+1, length(dx0));
    [x,dx] = computeOpenloopSolution(system, N, T, t0, x0, dx0, du);

    c = [];
    ceq = [];
    
    for k=1:N
        [cnew, ceqnew] = constraints(t0+k*T,dx(k,:),du(:,k));
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,dx(N+1,:));
    c = [c cnew];
    ceq = [ceq ceqnew];


end
end
