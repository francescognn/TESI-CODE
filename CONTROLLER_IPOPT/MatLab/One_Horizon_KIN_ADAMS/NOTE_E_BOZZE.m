traj; 
q = Xcomp(1:9,5);%[x;y;th;ones(6,1)*pi/tf*tt];
j=1;

for j=1:size(q,2)
    [T,P] = TeP(q(1:9,j));
    plot3(P(1,3:4),P(2,3:4),P(3,3:4),'b','LineWidth',5)
    hold on
    plot3(P(1,5:end),P(2,5:end),P(3,5:end),'r','LineWidth',4)
    grid on
    axis([-3 3 -3 3 0 2])
    hold off
    pause(0.03)
end

    for k=1:size(Xcomp,2)
       
       T=jacobian_MM_simple_val(Xcomp(1:9,k));
        X_ee_calc(:,k)=T(1:3,4);
        
    end
    
    X_ee_sol = Xcomp(10:12,:);
    er=X_ee_sol-X_ee_calc;
 
    
uint=zeros(8,1);

for k=1:size(unew,2)

    uint=[uint uint(:,end)+unew(:,k).*0.3];
    
end

syms x_ee y_ee th u t dt r

k1=[-r*sin(th);r*cos(th);1].*u;

k2 = [-r*sin(th+dt/2*k1(3));r*cos(th);1].*u;
k3 = [-r*sin(th+dt/2*k2(3));r*cos(th);1].*u;
k4 = [-r*sin(th+dt*k3(3));r*cos(th);1].*u;

xstep = [x_ee y_ee th].'+dt/6.0*(k1+2*k2+2*k3+k4);

