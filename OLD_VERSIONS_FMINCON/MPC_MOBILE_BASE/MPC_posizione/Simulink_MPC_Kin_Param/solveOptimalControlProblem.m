
function [p, V, xN] = solveOptimalControlProblem ...
    (runningcosts, terminalcosts, constraints, terminalconstraints, ...
    linearconstraints, system,controlaction, N, t0, x0, p0, T, xs, xd,j,xNedprima)

% DESCRIZIONE DI solveOptimalControlProblem
% la funzione prende in ingresso: 
% 
% > runningunningcosts: � la parte della funzione di costo che va valutata per 
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
% > system: � una funzione che calcola come propagare lo stato (x e dx):
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
% > xs: � lo stato desiderato modificato per tenere in considerazione l'obtacle avoidance
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
    
    [x] = computeOpenloopSolution(system,controlaction, N, T, t0, x0, p0, xNedprima);

    for k=1:N
        [Anew, bnew, Aeqnew, beqnew, lbnew, ubnew] = ...
               linearconstraints(t0+k*T,x(k,:),p0);
        A = blkdiag(A,Anew);
        b = [b, bnew];
        Aeq = blkdiag(Aeq,Aeqnew);
        beq = [beq, beqnew];
        lb = [lb, lbnew];
        ub = [ub, ubnew];
    end
    
    
[p V] = fmincon(@(p) costfunction(runningcosts, terminalcosts, system, ... 
     N, T, t0, x0, p, xs, xd), p0, A, b, Aeq, beq, lb, ub, @(p) ...
     constraintseval(constraints,terminalconstraints, system, N, T, t0, x0, xd, p,p0,j));

XN=computeOpenloopSolution(system,controlaction, N, T, t0, x0, p, xNedprima);

xN=XN(end,:);

function cost = costfunction(runningcosts, terminalcosts, system, ...
                    N, T, t0, x0, p, xs, xd)
cost = 0;
    x  = zeros(N+1, length(x0));
    x = computeOpenloopSolution(system, controlaction, N, T, t0, x0, p, xNedprima);
    
    for k=1:N
        cost = cost+runningcosts(k,N, x(k,:), xs(k,:), xd(k,:),p);
    end
    %cost = cost+terminalcosts(t0+(N+1)*T, x(N+1,:),xs(N+1,:));

end


function [c,ceq] = constraintseval ...
        (constraints, terminalconstraints, system, N, T, t0, x0, xd, p,p0, j)
    
    x  = zeros(N+1, length(x0));
   [x] = computeOpenloopSolution(system, controlaction, N, T, t0, x0, p, xNedprima);

    c = [];
    ceq = [];
    
    for k=1:N
        % abbiamo cambiato, prima questa riga sotto non c'era e constraints
        % era valutato su u(:,k)
%         du=u(:,k)-u(:,k-1);
        tt=k*T;
        [cnew, ceqnew] = constraints(xd(k,:),x(k,:),p,p0,tt);
        c = [c cnew];
        ceq = [ceq ceqnew];
    end
    [cnew, ceqnew] = terminalconstraints( xd ,x(N+1,:), p, N, T);
    c = [c cnew];
    ceq = [ceq ceqnew];
end

function [x] = computeOpenloopSolution(system, controlaction, N, T, t0, x0, p, xNedprima)
    
    x=zeros(1,size(x0,2));
    x(1,:) = x0;
    
    [u] = controlaction(T,N,p,xNedprima(1,:),xNedprima(2,:));
    for k=1:N
    x(k+1,:) = system(t0+T*k,x(k,:), u(:,k),T);
    end

end

end
