%% Definizioni

clear all
close all
clc

T=0.25;
Npoint=7;
t_max=10;
 [tt,ss,null1,null2]=get_traj(T,20);

%[ss,xmeasure0,tt] = traj_gen(T,t_max,Npoint);

N=6;
%xmeasure=[0,0,0];
tt=[tt tt(end)+T];
xd=[ss ; ss(end-2:end,:)];
xs=xd;
p0=zeros(1,7);
tmeasure=0;
u0=zeros(2,N);
xmeasure=xd(1,:);
umeasure=u0;
t0=0;


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
    p = [];
    
 
  ind_fin=1;
    
 for k=1:length(xd)-1

    [u0, x0] = measureInitialValue ( umeasure, xmeasure );
    %[xs,xd]=feasibility
    
    if k>=length(xd)-N+1
        
    x=[ x ;  xmeasure ];
    
        for jj=1:N
        
         tt(jj)=T*(jj-1);
   
         u_new(1,jj) = tretratti(tt(jj),N*T,p0,T);
         u_new(2,jj) = p0(4)*tt(jj)^3+p0(5)*tt(jj)^2+p0(6)*tt(jj)+p0(7);
        
         
         end
        
    
    [xmeasure]=system(t,x0,u0(:,ind_fin),T);   
    
    u=[u u0(:,ind_fin)]; 

     if k==length(xd)-1
    
        x=[ x ;  xmeasure ];
    
     else
     end
    
    ind_fin=ind_fin+1;
    
    else
        
    xsciclo=xs(k:k+N,:);
    xdciclo=xd(k:k+N,:);
    
    t_start=tic;
  
    % Solutore
    
 [p_new, V] = solveOptimalControlProblem ...
    (@runningcosts, @terminalcosts, @constraints, @terminalconstraints, ...
    @linearconstraints, @system, N, t0, x0, p0, T, xsciclo, xdciclo, options);

t_Elapsed(k) = toc( t_start );

        t = [ t; tmeasure ];
        %
        
        x=[ x ;  xmeasure ];
        
         for jj=1:N
        
         tt(jj)=T*(jj-1);
   
         u_new(1,jj) = tretratti(tt(jj),N*T,p_new,T);
         u_new(2,jj) = p_new(4)*tt(jj)^3+p_new(5)*tt(jj)^2+p_new(6)*tt(jj)+p_new(7);
        
         
         end
        
        u=[ u   u_new(:,1) ];
        p=[ p ; p_new];
        
       
        
        [u0] = shiftHorizon(u_new);
        
        [xmeasure]=system(t,x0,u_new,T);
        umeasure=u_new;
        p0=p_new;
    end      
        %% plot
        
        figure(1);
        plot(xd(:,1),xd(:,2),'--*k');
        title('x_1/x_2 closed loop trajectory');
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        p1 = [x(end,1) x(end,2)];                         % First Point
        p2 = [x(end,1)+0.2*cos(x(end,3)) x(end,2)+0.2*sin(x(end,3))];                         % Second Point
        dp = p2-p1;                         % Difference
        quiver(p1(1),p1(2),dp(1),dp(2),'-*r','LineWidth',5,'MarkerSize',1,'ShowArrowHead','off')
        p11 = [x(end,1) x(end,2)];                         % First Point
        p22 = [x(end,1)-0.2*cos(x(end,3)) x(end,2)-0.2*sin(x(end,3))];                         % Second Point
        dp = p22-p11;                         % Difference
        quiver(p11(1),p11(2),dp(1),dp(2),'-g','LineWidth',5,'ShowArrowHead','off')
        axis([-1 max(xd(:,1))*1.15 -1 max(xd(:,2))*1.15]);
        viscircles([4 -0.1],0.3)% obstacle avoidance draw
        
        hold off
        
        FF(k)=getframe(gcf);
        
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

figure(2)
plot(u')
grid on
legend('u1','u2')

figure(3)
plot(x)
grid on
legend('x','y','th')

figure(4)
subplot(411)
plot(x(:,1)-xd(:,1))
title('error on X')
grid on

subplot(412)
plot(x(:,2)-xd(:,2))
title('error on Y')
grid on

subplot(413)
plot(rad2deg(x(:,3)-xd(:,3)))
title('Error on Theta')
grid on

subplot(414)
plot(sqrt(xd(:,1).^2+xd(:,2).^2)-sqrt(x(:,1).^2+x(:,2).^2))
title('Error NORM XY')
grid on

figure(5)
plot(t_Elapsed)
title('t elapsed')
grid on

% video= VideoWriter('Simulazione_MPC_NL_kin.avi','Uncompressed AVI');
% open(video)
% writeVideo(video,FF)
% close(video)

 %% VARIABILI DEL PROBLEMA

function cost = runningcosts(t, x, xs, xd)

Q=eye(3)*2000;
%Q(3,3)=5;
R=diag([5 5]);
T=eye(2)*1;

cost = [x-xs]*Q*[x-xs]';%+u.'*R*u;%+(xs-xd)*T*(xs-xd)';

end

function cost = terminalcosts(t, x, xs)

    P=eye(3)*50;
    cost = [x-xs]*P*[x-xs]';
    
end

function [y] = system(t, x, u, T)

    

    y(:,1)=x(1,1)+T*u(1,1)*cos(x(1,3));
    y(:,2)=x(1,2)+T*u(1,1)*sin(x(1,3));
    y(:,3)=x(1,3)+T*u(2,1);
        
end

function [c,ceq] = constraints(t, x, u,N,j)
    c   = []; % 0.09-(x(:,1)-4).^2-(x(:,2)+0.1).^2]; % vincolo cerchio
    ceq = [];
    
end

function [c,ceq] = terminalconstraints(xd, x, p, N, T)
% ricorda che il vincolo finale su x è veramente finale, quello su u è il
% controllo al momento prima
    c   = [];
    ceq = [x(1)-xd(end,1); x(2)-xd(end,2); x(3)-xd(end,3); tretratti(N*T,N*T,p,T) ];

end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [ -6*ones(1,7) ];
    ub  = [ 6*ones(1,7) ];
end

%% ALTRE DEFINIZIONI

function [u0, x0] = measureInitialValue ( umeasure, xmeasure )

    u0 = umeasure;
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