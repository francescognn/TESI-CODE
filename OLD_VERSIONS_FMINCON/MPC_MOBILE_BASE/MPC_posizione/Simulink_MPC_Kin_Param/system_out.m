function [y] = system_out(t, x, u, T)

    y=[];

    y(:,1)=x(1,1)+T*u(1,1)*cos(x(1,3));
    y(:,2)=x(1,2)+T*u(1,1)*sin(x(1,3));
    y(:,3)=x(1,3)+T*u(2,1);
    
end