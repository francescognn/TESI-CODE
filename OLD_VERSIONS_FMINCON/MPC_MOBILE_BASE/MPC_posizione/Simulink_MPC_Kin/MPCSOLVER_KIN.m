function [u_new, V] = MPCSOLVER_KIN(N, t0, x0, u0, T, xs, xd,j)

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

Q=eye(size(xs,2))*4e6;
Q(3,3)=1000;
R=eye(size(u,1))*1;
Ti=eye(size(xs,2))*1;

cost = 0.5*[(x(:,1:size(xs,2))-xs)]*Q*[(x(:,1:size(xs,2))-xs)].'+0.5*(xs-xd)*Ti*(xs-xd).';
%cost = 0.5*[norm(x(:,2)-xs(1:2))]*4e3*[norm(x(:,2)-xs(1:2))].'+x(:,3)*4e2*x(:,3);%
end

function cost = terminalcosts(t, x, xs)

    P=eye(size(xs,2))*2000000;
    cost = 0.5*[(x(:,1:size(xs,2))-xs)]*P*[(x(:,1:size(xs,2))-xs)]';
end

function [y] = system(x0, x, u, T)
       % x0 lo ho messo così possiamo provare la linearizzazione tutta sullo stato 0 di ogni problema di ottimizzazione. 
    y(:,1)=x(1,1)+T*u(1,1)*cos(x(1,3));
    y(:,2)=x(1,2)+T*u(1,1)*sin(x(1,3));
    y(:,3)=x(1,3)+T*u(2,1);
    
end

function [c,ceq] = constraints(du, x, u, N, j,xd)
    c   = [du(1)-0.15 ; du(2)-0.07];
    ceq = [];
    
    if j>0
    c=[du(1)-0.15 ; du(2)-0.07];
    ceq = [x(1)-xd(1) ; x(2)-xd(2) ; x(3)-xd(3)];
    end
end

function [c,ceq] = terminalconstraints(xd,x,u)
    c   = [];
    ceq = [ x(:,1)-xd(end,1) ; x(:,2)-xd(end,2) ; x(:,3)-xd(end,3) ];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -3 ;-6 ];
    ub  = [  3 ; 6 ];
end
end

