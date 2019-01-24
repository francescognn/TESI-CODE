function [u_new, V] = MPCSOLVER(N, t0, x0, u0, T, xs, xd,j)

options = optimset('Display','off',...
                'TolFun', 1e-6,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);


[u_new, V] = solveOptimalControlProblem ...
        (@runningcosts, @terminalcosts, @constraints, @terminalconstraints, ...
        @linearconstraints, @system, N, t0, x0, u0, T, xs, xd,j,options);


function cost = runningcosts(t, u, x, xs, xd)

Q=eye(size(xs,2))*400;
R=eye(size(u,1))*1;
Ti=eye(size(xs,2))*1;

cost = 0.5*[(x(:,1:size(xs,2))-xs)]*Q*[(x(:,1:size(xs,2))-xs)].'+0.5*(xs-xd)*Ti*(xs-xd).';

end

function cost = terminalcosts(t, x, xs)

    P=eye(size(xs,2))*2000;
    cost = 0;%.5*[(x(:,1:size(xs,2))-xs)]*P*[(x(:,1:size(xs,2))-xs)]';
end

function [y] = system(t, x, u, T)
            
        [A,F,D] = matriceseval(x);
        
         dx=A*x.'+ F*u(:,1)+D;  
         y = (x.'+ T*dx).';
end

function [c,ceq] = constraints(du, x, u, N, j)
    c   = [];%du(1)-0.9;du(2)-0.8];
    ceq = [];
%     if j>0
%     c=[];
%     ceq = [%x(1)-xd(1); x(2)-xd(2); x(3)-xd(3); x(4); x(5)];
%     end
end

function [c,ceq] = terminalconstraints(xd,x,u)
    c   = [];
    ceq = [];%x(:,1)-xd(end,1);x(:,2)-xd(end,2);x(:,3)-xd(end,3); x(:,4) ; x(:,5)];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -30 ; -30 ];
    ub  = [ 30 ; 30 ];
end
end

