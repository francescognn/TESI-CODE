function [u] = controlaction_out(T,N,p,xN,xdp)

   for k=1:N     
        if p(end)>=k
             
                tt(k)=T*(k-1);

                u(1,k) = p(1)*tt(k)^2+p(2)*tt(k)+p(3); %tretratti(tt(k),N*T,p,T);
                u(2,k) = p(4)*tt(k)^2+p(5)*tt(k)+p(6);%*tt(k)+p(7);            
        else
    
        B = [cos(xN(3)) 0; ...
             sin(xN(3)) 0; ...
               0     1];
        Q=200*ones(3);
  
   % ricorda che xdp è xdN+1
   
        G=2*B.'*Q*xN.'-B.'*Q*xdp.'-B.'*xdp.';
        F=2*B*Q*B.';

        u=-G\F;
    
        end
   end
end

