function [u_new, V] = MPCSOLVER_xp_yp(N, t0, x0, u0, T, xs, xd,j)

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

Q=[2000 0; 0 2000];
% R=eye(2)*30;
Ti=eye(2)*1;

cost = [(x(:,1)-xs(:,1)) (x(:,2)-xs(:,2))]*Q*[(x(:,1)-xs(:,1)) (x(:,2)-xs(:,2))]'+(xs-xd)*Ti*(xs-xd)';

end

function cost = terminalcosts(t, x, xs)

    P=[5000 0; 0 5000];
    cost = [(x-xs)]*P*[(x-xs)]';
end

function [y] = system(t, x, u, T)
        
    y(1) = x(1)+T*u(1);
    y(2) = x(2)+T*u(2);
end

function [c,ceq] = constraints(du, x, u, N, j)
    c   = [];%[du(1)-1.5;du(2)-0.03];
    ceq = [];
%     if j>0
%     c=[];
%     ceq = [];%x(1)-xd(1); x(2)-xd(2)];
%     end
end

function [c,ceq] = terminalconstraints(xd,x,u)
    c   = [];
    ceq = [x(:,1)-xd(end,1);x(:,2)-xd(end,2)];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -4 ; -4 ];
    ub  = [ 4 ; 4 ];
end
end

