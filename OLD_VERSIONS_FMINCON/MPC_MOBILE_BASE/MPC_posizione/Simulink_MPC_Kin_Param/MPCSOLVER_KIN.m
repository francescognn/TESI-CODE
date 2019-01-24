function [p_new, V, xN] = MPCSOLVER_KIN(N, t0, x0, p0, T, xs, xd,j,m,xNedprima)

options = optimset('Display','off',...
                'TolFun', 1e-6,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);


[p_new, V, xN] = solveOptimalControlProblem ...
        (@runningcosts, @terminalcosts, @constraints, @terminalconstraints, ...
        @linearconstraints, @system, @controlaction, N, t0, x0, p0, T, xs, xd,j, xNedprima);


function cost = runningcosts(k, N, x, xs, xd, p)

Q=eye(size(xs,2))*20000;
%R=eye(size(u,1))*1;
Ti=eye(size(xs,2))*1;

%ke=lN_star/l1_star;
%-log(2*ke)/log(N);

cost =  ((k/N)^m)*[(x(:,1:size(xs,2))-xs)]*Q*[(x(:,1:size(xs,2))-xs)].';%+0.5*(xs-xd)*Ti*(xs-xd).';%+300000*abs([p(6) p(5) p(4)]*[1 2*t 3*t.^2].').^2;

end

function cost = terminalcosts(t, x, xs)

    P=eye(size(xs,2))*20;
    cost = 0;%0.5*[(x(:,1:size(xs,2))-xs)]*P*[(x(:,1:size(xs,2))-xs)]';
end

function [y] = system(t, x, u, T)
       
    y(:,1)=x(1,1)+T*u(1,1)*cos(x(1,3));
    y(:,2)=x(1,2)+T*u(1,1)*sin(x(1,3));
    y(:,3)=x(1,3)+T*u(2,1);
    
end

function [c,ceq] = constraints(xd, x, p, p0, t)
    c   = [abs([p(3) p(2) p(1)]*[0 1 2*t].')-2.5;  abs([p(6) p(5) p(4)]*[0 1 2*t].')-2.5];
    ceq = [];
    
%     if j>0
%     c=[du(1)-0.6;du(2)-0.15];
%     ceq = [x(1)-xd(1); x(2)-xd(2); x(3)-xd(3)];
%     end
end

function [c,ceq] = terminalconstraints(xd,x,p,N,T)
    c   = [];
    ceq = [];%x(:,1)-xd(end,1);x(:,2)-xd(end,2);x(:,3)-xd(end,3)]; %tretratti(N*T,N*T,p,T)];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, p)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];
end

function [u] = controlaction(T,N,p,xN,xdp)
   
   for k=1:N     
        if p(end)>=k
             
                tt(k)=T*(k-1);

                u(1,k) = p(1)*tt(k)^2+p(2)*tt(k)+p(3); %tretratti(tt(k),N*T,p,T);
                u(2,k) = p(4)*tt(k)^2+p(5)*tt(k)+p(6);%*tt(k)+p(7);            
        else
    
        B = [cos(xN(3)) 0; ...
             sin(xN(3)) 0; ...
               0     1];
        Q=20000*ones(3);
  
   % ricorda che xdp è xdN+1
   
        G=2*B.'*Q*xN.'-B.'*Q*xdp.'-B.'*xdp.';
        F=2*B*Q*B.';

        u=-G\F;
    
        end
   end
end



end

