%% Definizioni

clear all 
close all
clc

T=0.2;
Npoint=4;

Ttot=5;
[ss, ss_p] = trajectory_ee(T,Ttot);
ss = [zeros(size(ss,1),3) , ss];
tt = 0:T:Ttot;

N=6;
%xmeasure=[0,0,0];
tt=[tt tt(end)+T];
xd=[ss ; ss(end,:)];
xs=xd;
u0=zeros(8,N);
tmeasure=0;
q0 = [ 0 0 0 0 pi/2 pi/2 pi/4 pi/4 pi/4]; 
xmeasure= [q0 forw_kin(q0).'];

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
v = [];
q = [];

ind_fin=1;
    
 for k=1:length(xd)-1

    [t0, x0] = measureInitialValue ( tmeasure, xmeasure );
    %[xs,xd]=feasibility
    
    if k>=length(xd)-N+1
        
        x=[ x ;  xmeasure ];

        [xmeasure]=system(t,x0,u0(:,ind_fin),T);   

        u=[u u0(:,ind_fin)]; 

        if k==length(xd)-1

            x=[ x ;  xmeasure ];
        end
    
    ind_fin=ind_fin+1;
    
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

    x=[ x ;  xmeasure ];

    u=[ u   u_new(:,1) ];

    [u0] = shiftHorizon(u_new);

    [xmeasure]=system(t,x0,u_new,T);
    end      
%         %% plot
%         
%         figure(1);
%         plot(xd(:,1),xd(:,2),'--*k');
%         title('x_1/x_2 closed loop trajectory');
%         xlabel('x_1');
%         ylabel('x_2');
%         grid on;
%         hold on;
%         p1 = [x(end,1) x(end,2)];                         % First Point
%         p2 = [x(end,1)+0.2*cos(x(end,3)) x(end,2)+0.2*sin(x(end,3))];                         % Second Point
%         dp = p2-p1;                         % Difference
%         quiver(p1(1),p1(2),dp(1),dp(2),'-*r','LineWidth',5,'MarkerSize',1,'ShowArrowHead','off')
%         p11 = [x(end,1) x(end,2)];                         % First Point
%         p22 = [x(end,1)-0.2*cos(x(end,3)) x(end,2)-0.2*sin(x(end,3))];                         % Second Point
%         dp = p22-p11;                         % Difference
%         quiver(p11(1),p11(2),dp(1),dp(2),'-g','LineWidth',5,'ShowArrowHead','off')
%         axis([-1 max(xd(:,1))*1.15 -1 max(xd(:,2))*1.15]);
%         %viscircles([4 -0.1],1) obstacle avoidance draw
%         
%         hold off
        
 end 
%  vp=diff(u(1,:))./T;
%  omegap=diff(u(2,:))./T;
%  Up=[tt(:,1:end-2).' vp.' omegap.'];
%  U=[tt(:,1:end-2).' u(:,2:end).'; ((tt(end)):0.2:40).' zeros(length((tt(end)):0.2:40),2)];
%  Up=[ Up; ((tt(end)):0.2:40).' zeros(length(tt(end):0.2:40),2)];
%  
%  U=[0 0 0 ; U(2:end,1) U(1:end-1,2) U(1:end-1,3)];
%  Up=[0 0 0 ; Up(2:end,1) Up(1:end-1,2) Up(1:end-1,3)];
%  limx=[-1 max(xd(:,1))*1.15]; 
%  limy=[-1 max(xd(:,2))*1.15];
 
%% PLOT CONCLUSIVI

% figure(2)
% plot(u')
% grid on
% legend('u1','u2')
% 
% figure(3)
% plot(x)
% grid on
% legend('x','y','th')
% 
% figure(4)
% subplot(411)
% plot(x(:,1)-xd(:,1))
% title('error on X')
% grid on
% 
% subplot(412)
% plot(x(:,2)-xd(:,2))
% title('error on Y')
% grid on
% 
% subplot(413)
% plot(rad2deg(x(:,3)-xd(:,3)))
% title('Error on Theta')
% grid on
% 
% subplot(414)
% plot(sqrt(xd(:,1).^2+xd(:,2).^2)-sqrt(x(:,1).^2+x(:,2).^2))
% title('Error NORM XY')
% grid on
% 
% figure(5)
% plot(t_Elapsed)
% title('t elapsed')
% grid on

 %% VARIABILI DEL PROBLEMA

function cost = runningcosts(t, u, x, xs, xd)

Q=eye(6)*2000;
R=eye(8)*5;
T=eye(9)*1;

cost = [[x(:,1:3) , x(:,10:12)]-xs]*Q*[[x(:,1:3) , x(:,10:12)]-xs]'+u.'*R*u;%+(xs-xd)*T*(xs-xd)';

end

function cost = terminalcosts(t, x, xs)
    P=eye(6)*0;
    cost = [[x(:,1:3) , x(:,10:12)]-xs]*P*[[x(:,1:3) , x(:,10:12)]-xs]';
    
end

function [y] = system(t, x, u, T)
    th=x(3);
    G = [cos(th), 0 ; 
         sin(th), 0 ; 
         0,       1 ];
    J = jacobian_MM(x(1:9));
    P = [G , zeros(3,6); zeros(6,2), eye(6); J(1:6,1:8)];
    x=x.';
    y(1:15,1) = x(1:15,1) + T.*P*u(1:8,1);
    y=y.';
        
end

function [c,ceq] = constraints(t, x, u,N,j)
    c   = [x(:,4:9).' - pi; - x(:,4:9).' - pi];%[1-(x(:,1)-4).^2-(x(:,2)+0.1).^2]; % vincolo cerchio
    ceq = [];
    
end

function [c,ceq] = terminalconstraints(xd, x, u)
    c   = [];
    ceq = [];%x(1)-xd(end,1) x(2)-xd(end,2) x(3)-xd(end,3) u(1) u(2)];

end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -10 ; -10 ; -pi ; -pi ; -pi ; -pi ; -pi ; -pi];
    ub  = [ 10 ; 10 ; pi ; pi ; pi ; pi ; pi ; pi];
end

%% ALTRE DEFINIZIONI

function [t0, x0] = measureInitialValue ( tmeasure, xmeasure )

    t0 = tmeasure;
    x0 = xmeasure;

end

function [u0] = shiftHorizon(u)

    u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
    
end

%% NOTE
% 
% bisogna che capiamo come mettere i vincoli sull'accelerazione.
% devo passare al solutore anche il controllo al passo precedente in qualche modo. 
% 
% per il resto si dovrebbe poter fare easy