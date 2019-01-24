%% Definizioni

clear all 
close all
clc

T=0.5;
epsilon=0.15;

[tt,ss,null1,null2]=get_traj(T,20);

N=4;
xmeasure=[0,0];
xd(:,1)=ss(:,1)+epsilon*cos(ss(:,3));
xd(:,2)=ss(:,2)+epsilon*sin(ss(:,3));

xs=xd;
u0=[zeros(2,5)];
tmeasure=0;
q1=zeros(1,3);
v1=zeros(2,1);


%% dati solutore

options = optimset('Display','off',...
                'TolFun', 1e-5,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);

warning off all

%% MPC ALGORITHM 
    t = [];
    x = [];
    u = [];
    v = [];
    q = [];
 
    
 for k=1:length(xd)

     [t0, x0, q0] = measureInitialValue ( tmeasure, xmeasure, q1 );
    %[xs,xd]=feasibility
    
    if k>=length(xd)-N+1
        
    x = [ x ;  xmeasure ];
    q = [q ; q1];
    
    [xmeasure,q1,v1]=systemkin(t,q0,x0,u0(:,k-(length(xd)-N)),T,epsilon);   
    u=[u u0(:,k-(length(xd)-N))]; 
    v=[v v1];
%     xsciclo = [xs(k:length(xd),:) ; ones(k+N-length(xd),2).*xs(end,:)];
%     xdciclo = [xd(k:length(xd),:) ; ones(k+N-length(xd),2).*xd(end,:)];
    else
    xsciclo=xs(k:k+N,:);
    xdciclo=xd(k:k+N,:);
    
    
    t_start=tic;
  
    % Solutore
    
 [u_new, V] = solveOptimalControlProblem ...
    (@runningcosts, @terminalcosts, @constraints, @terminalconstraints, ...
    @linearconstraints, @system, N, t0, x0, u0, T, xsciclo, xdciclo, options);

t_Elapsed(k) = toc( t_start );

        t = [ t; tmeasure ];
        %
        
        x=[ x ;  xmeasure ];
        q=[ q ; q0];
        u=[ u   u_new(:,1) ];
        v=[v v1];
       
        
        [u0] = shiftHorizon(u_new);
        
        [xmeasure,q1,v1]=systemkin(t,q0,x0,u_new,T,epsilon);
    end      
        %% plot
        
        figure(1);
        plot(ss(:,1),ss(:,2),'--*k');
        title('x_1/x_2 closed loop trajectory');
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        p1 = [q(end,1) q(end,2)];                         % First Point
        p2 = [q(end,1)+0.2*cos(q(end,3)) q(end,2)+0.2*sin(q(end,3))];                         % Second Point
        dp = p2-p1;                         % Difference
        quiver(p1(1),p1(2),dp(1),dp(2),'-*r','LineWidth',5,'MarkerSize',1,'ShowArrowHead','off')
        p11 = [q(end,1) q(end,2)];                         % First Point
        p22 = [q(end,1)-0.2*cos(q(end,3)) q(end,2)-0.2*sin(q(end,3))];                         % Second Point
        dp = p22-p11;                         % Difference
        quiver(p11(1),p11(2),dp(1),dp(2),'-g','LineWidth',5,'ShowArrowHead','off')
        axis([-1 max(ss(:,1))*1.15 -1 max(ss(:,2))*1.15]);
        hold off
 end 
 
%% PLOT CONCLUSIVI

figure(2)
plot(u')
grid on
legend('u1','u2')

figure(3)
plot(q)
grid on
legend('x1','x2')

figure(4)
subplot(311)
plot(q(:,1)-ss(:,1))
title('error on x')
grid on

subplot(312)
plot(q(:,2)-ss(:,2))
title('error on y')
grid on

subplot(313)
plot(sqrt(ss(:,1).^2+ss(:,2).^2)-sqrt(q(:,1).^2+q(:,2).^2))
title('Error NORM')
grid on

figure(5)
title('t_elapsed')
plot(t_Elapsed)
grid on

 %% VARIABILI DEL PROBLEMA

function cost = runningcosts(t, u, x, xs, xd)

Q=[20000 0; 0 2000];
R=eye(2)*30;
T=eye(2)*1;

cost = [(x(:,1)-xs(:,1))+0.13 (x(:,2)-xs(:,2))]*Q*[(x(:,1)-xs(:,1))+0.13 (x(:,2)-xs(:,2))]'+(xs-xd)*T*(xs-xd)';

end

function cost = terminalcosts(t, x, xs)

    P=[5000 0; 0 5000];
    cost = [(x-xs)]*P*[(x-xs)]';
end

function [y] = system(t, x, u, T)
        
        y(1) = x(1,1)+T*u(1,1);
        y(2) = x(1,2)+T*u(2,1);
        
end

function [c,ceq] = constraints(t, x, u)
    c   = [];
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x, u)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -4 ; -4 ];
    ub  = [ 4 ; 4 ];
end

%% ALTRE DEFINIZIONI

function [t0, x0, q0] = measureInitialValue ( tmeasure, xmeasure, q1 )
    t0 = tmeasure;
    x0 = xmeasure;
    q0 = q1;
end

function [u0] = shiftHorizon(u)

    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
    
end

function [y,q1,v] = systemkin(t,q0,x,u,T,epsilon)

y(1) = x(1,1)+T*u(1,1);
y(2) = x(1,2)+T*u(2,1);

xp=x(:,1);yp=x(:,2);

xq=q0(:,1);
yq=q0(:,2);
thq=q0(:,3);


A=[cos(thq) sin(thq); -1/epsilon*sin(thq) 1/epsilon*cos(thq)];

v=A*[u(1,1);u(2,1)];

q1(:,1)=xq+T*v(1,1)*cos(thq);
q1(:,2)=yq+T*v(1,1)*sin(thq);
q1(:,3)=thq+T*v(2,1);


end

%% NOTE
% 
% bisogna che capiamo come mettere i vincoli sull'accelerazione.
% devo passare al solutore anche il controllo al passo precedente in qualche modo. 
% 
% per il resto si dovrebbe poter fare easy