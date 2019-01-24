%% Definizioni

clear all 
close all
clc

ss=get_pencil_curve;

N=3;
xmeasure=[10,10];
xd=ss*100;
xs=xd;
dxmeasure=[0,0];
u0=[0; 0];
dumeasure=[u0 zeros(2,N-1)];
du0=dumeasure;
tmeasure=0;
T=2;

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
    dx = [];
    du = [];
    
 for k=1:length(xd)

    [t0, x0, dx0] = measureInitialValue ( tmeasure, xmeasure, dxmeasure );
    %[xs,xd]=feasibility
    
    if k>=length(xd)-N+1
    xsciclo = [xs(k:length(xd),:) ; ones(k+N-length(xd),2).*xs(end,:)];
    xdciclo = [xd(k:length(xd),:) ; ones(k+N-length(xd),2).*xd(end,:)];
    else
    xsciclo=xs(k:k+N,:);
    xdciclo=xd(k:k+N,:);
    end
    
    t_start=tic;
  
    % Solutore
    
 [du_new, V] = solveOptimalControlProblem ...
    (@runningcosts, @terminalcosts, @constraints, @terminalconstraints, ...
    @linearconstraints, @system, N, t0, x0, dx0, du0, T, xsciclo, xdciclo, options);

t_Elapsed = toc( t_start );

        t = [ t; tmeasure ];
        dx = [ dx; dxmeasure ];
        du = [ du du_new(:,1)];
        %
        if k==1
        
        x=[ x ;  xmeasure ];
        u=[ u ;  dumeasure ];
        
        else
           
        x=[ x ; x(end,:) + dxmeasure ];
        u=[ u  u(:,end) + du_new(:,1) ];
        end
        
        [u0 du0] = shiftHorizon(u,du_new);
        
        [xmeasure, dxmeasure]=system(t,x0,dx0,du_new,T);
        
        %% plot
        
        figure(1);
        plot(xd(:,1),xd(:,2),'--*k');
        title('x_1/x_2 closed loop trajectory');
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        plot(x(:,1),x(:,2),'or', ...
             'MarkerFaceColor','r','markersize',4);
        axis([-10 max(xd(:,1))*1.15 -10 max(xd(:,2))*1.15]);
        axis square;
 end 
 
%% PLOT CONCLUSIVI

figure(2)
plot(u')
grid on
legend('u1','u2')

figure(3)
plot(x)
grid on
legend('x1','x2')

figure(4)
plot(du')
grid on
hold on
legend('du1','du2')
ylim([-12 12]);
plot([ones(1,size(du,2))*10])
plot([ones(1,size(du,2))*-10]) 

%% VARIABILI DEL PROBLEMA

function cost = runningcosts(t, dx, du, x, xs, xd)

Qx=eye(2)*100;
Qe=eye(2)*20;
Q=[Qx,zeros(2);zeros(2),Qe];
R=eye(2)*30;
T=eye(2)*1;

cost = [dx,(x-xs)]*Q*[dx,(x-xs)]'+du'*R*du+(xs-xd)*T*(xs-xd)';

end

function cost = terminalcosts(t, x, dx, xs)

    P=[1 0 0 0; 0 1 0 0; 0 0 50 0; 0 0 0 50];
    cost = [dx,(x-xs)]*P*[dx,(x-xs)]';
end

function [y,dy] = system(t, x, dx, du, T)
        
        dy(1) = dx(1,1)+T*du(1);
        dy(2) = dx(1,2)+T*du(2);
        
        y(1) = x(1,1)+dy(1);
        y(2) = x(1,2)+dy(2);
        
        
end

function [c,ceq] = constraints(t, dx, du)
    c   = [];
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, dx)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = -10;
    ub  = +10;
end

%% ALTRE DEFINIZIONI

function [t0, x0, dx0] = measureInitialValue ( tmeasure, xmeasure, dxmeasure )
    t0 = tmeasure;
    x0 = xmeasure;
    dx0=dxmeasure;
end

function [u0, du0] = shiftHorizon(u,du)

    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
    du0 = [du(:,2:size(du,2)) du(:,size(du,2))];
    
end




