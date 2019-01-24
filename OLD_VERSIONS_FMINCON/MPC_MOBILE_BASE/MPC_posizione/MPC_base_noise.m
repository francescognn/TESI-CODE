%% Definizioni

clear all 
close all
clc

T=0.3;

[tt,ss,null1,null2 ]=get_traj(T,20);

N=4;            %prediction horizon

xmeasure=[0,0,0,0,0];
xd=[ss];
xs=xd;
u0=[zeros(2,N)];
tmeasure=0;


%% dati solutore

options = optimset('Display','off',...
                'TolFun', 1e-8,...
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
 
    
 for k=1:length(xd)

    [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
    %[xs,xd]=feasibility
    
    if k>=length(xd)-N+1
        
    x=[ x ;  xmeasure ];
    u=[u u0(:,k-(length(xd)-N))];
    
    [xmeasure]=system2(t,x0,u0(:,k-(length(xd)-N)),T);   
        
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
        u=[ u   u_new(:,1) ];
       
        
        [u0] = shiftHorizon(u_new);
        
        [xmeasure]=system2(t,x0,u_new,T);
    end
        %% plot
        
        figure(1);
        plot(xd(:,1),xd(:,2),'--k');
        
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        p1 = [x(end,1) x(end,2)];                         % First Point
        p2 = [x(end,1)+0.25*cos(x(end,3)) x(end,2)+0.25*sin(x(end,3))];                         % Second Point
        dp = p2-p1;                         % Difference
        quiver(p1(1),p1(2),dp(1),dp(2),'-*r','LineWidth',5,'MarkerSize',1,'MaxHeadSize',15)
        p11 = [x(end,1) x(end,2)];                         % First Point
        p22 = [x(end,1)-0.25*cos(x(end,3)) x(end,2)-0.25*sin(x(end,3))];                         % Second Point
        dp = p22-p11;                         % Difference
        quiver(p11(1),p11(2),dp(1),dp(2),'-g','LineWidth',5,'MaxHeadSize',15)
        axis([-1 max(xd(:,1))*1.15 -1 max(xd(:,2))*1.15]);
        axis square;
       
 end 

%% PLOT CONCLUSIVI

figure(2)
plot(u')
grid on
legend('u1','u2')

figure(3)
plot(x(:,1:3))
grid on
legend('x','y','th')

figure(4)
plot(x(:,4:5))
grid on
legend('v','omega')

figure(5)
subplot(3,1,1)
plot(xd(:,1)-x(:,1))
grid on
ylabel('error x1');
subplot(3,1,2)
plot(xd(:,2)-x(:,2))
grid on
ylabel('error x2');
subplot(3,1,3)
plot(sqrt(xd(:,1).^2+xd(:,2).^2)-sqrt(x(:,1).^2+x(:,2).^2))
grid on
ylabel('error x3');


figure;
plot(t_Elapsed);
legend('t Elapsed');
grid on

 %% VARIABILI DEL PROBLEMA

function cost = runningcosts(t, u, x, xs, xd)

Q=eye(size(xs,2))*2000;
R=eye(size(u,1))*1;
T=eye(size(xs,2))*1;

cost = 0.5*[(x(:,1:size(xs,2))-xs)]*Q*[(x(:,1:size(xs,2))-xs)].'+0.5*(xs-xd)*T*(xs-xd).';

end

function cost = terminalcosts(t, x, xs)

    P=eye(size(xs,2))*2000;
    cost = 0.5*[(x(:,1:size(xs,2))-xs)]*P*[(x(:,1:size(xs,2))-xs)]';
end

function [y] = system2(t, x, u, T)
            
        [A,F,D] = matriceseval(x);
 
         dx=A*x.'+ F*u(:,1)+D;  
         y = (x.'+ T*dx).';        
%          y=(T.*(A+eye(size(A,1)))*x.'+T.*F*u(:,1)+T.*D).';
%         
end
function [y] = system(t, x, u, T)
            
        [A,F,D] = matriceseval(x);
        noise=-0.2;
        A=(1-noise)*A;
        F=(1-noise)*F;
        D=(1-noise)*D;
 
         dx=A*x.'+ F*u(:,1)+D;  
         y = (x.'+ T*dx).';        
%          y=(T.*(A+eye(size(A,1)))*x.'+T.*F*u(:,1)+T.*D).';
%         
end
function [c,ceq] = constraints(t, x, du)
    c   = [];%abs(du(1,:))-1;abs(du(2,:))-1];
    ceq = [];
end

function [c,ceq] = terminalconstraints(xd, x,u)
    c   = [];
    ceq = [x(:,1)-xd(end,1) ;  x(:,2)-xd(end,2) ; x(:,4) ; x(:,5)];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];
end

%% ALTRE DEFINIZIONI

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
end

function [u0] = shiftHorizon(u)

    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
    
end
