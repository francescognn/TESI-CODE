%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                   %
%      DISPLACEMENT AND ACCELERATIONS OF STANDARD CAM MOTIONS V1.0  %
%                                                                   %
%            Developed by Miguel Viegas Leal - 2016                 %
%                Universidade da Beira Interior                     %
%                          OpenSource                               %
%                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




function cam()
clc
clear all
close all

N_i = input('Number or periods in the motion of the cam:');

beta_i(1)=0;
L(1)=0;
displacement=L(1);

k=2;

    while(k<=N_i+1)
    
        k
        
        beta_i(k) = input('Beta angle of the motion:');
         
        L(k) = input('Lift of the motion:');
        
        displacement=displacement+L(k)
      
        mov(k) = input('Type of the motion:');

        k=k+1;

    end


R_0 = input('Raio base:');

teta=0;

step=0.1;
n=1;

T(n)=teta;

j=2;
Ae=[];
Te=[];
Pe=[];

while(j<=N_i+1)
    
    y=0;
    dy=0;
    teta=0;
    T=0;
    
    j-1
    
    while(teta<=beta_i(j))
        
        switch mov(j)
        
        case 1
      
            y=0.5*L(j)*(1-cos(pi*teta/beta_i(j)));
            dy=((pi*L(j))/(2*beta_i(j)))*sin(pi*teta/beta_i(j));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;

       case 2
            
            y=L(j)*((teta/beta_i(j))-0.5*(pi^-1)*sin(2*pi*(teta/beta_i(j))));
            dy=(L(j)/beta_i(j))*(1-cos(2*pi*(teta/beta_i(j))));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta;
            
        case 3
            
            y=0.5*L(j)*((1-cos(pi*teta/beta_i(j)))-0.25*(1-cos(2*pi*teta/beta_i(j))));
            dy=0.5*pi*(L(j)/beta_i(j))*(sin(pi*teta/beta_i(j))-0.5*sin(2*pi*teta/beta_i(j)));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta;
            
        case 4
            
            y=L(j)*(1-cos(0.5*pi*teta/beta_i(j)));
            dy=((pi*L(j))/(2*beta_i(j)))*sin(0.5*pi*teta/beta_i(j));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta; 
            
        case 5
            
            y=L(j)*sin(pi*0.5*teta/beta_i(j));
            dy=0.5*pi*(L(j)/beta_i(j))*cos(0.5*pi*teta/beta_i(j));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta; 
            
        case 6
            
            y=L(j)*((teta/beta_i(j))-(pi^-1)*sin(pi*(teta/beta_i(j))));
            dy=(L(j)/beta_i(j))*(1-cos(pi*(teta/beta_i(j))));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
        
        case 7
            
            y=L(j)*((teta/beta_i(j))+(pi^-1)*sin(pi*(teta/beta_i(j))));
            dy=(L(j)/beta_i(j))*(1+cos(pi*(teta/beta_i(j))));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
            
        case 8
 
            y=0.5*L(j)*(1+cos(pi*teta/beta_i(j)));
            dy=-((pi*L(j))/(2*beta_i(j)))*sin(pi*teta/beta_i(j));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta;
            
        case 9
 
            y=L(j)*(1-(teta/beta_i(j))-0.5*(pi^-1)*sin(2*pi*(teta/beta_i(j))));
            dy=-(L(j)/beta_i(j))*(1-cos(2*pi*(teta/beta_i(j))));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta;
            
        case 10
            
            y=0.5*L(j)*((1+cos(pi*teta/beta_i(j)))-0.25*(1-cos(2*pi*teta/beta_i(j))));
            dy=-0.5*pi*(L(j)/beta_i(j))*(sin(pi*teta/beta_i(j))+0.5*sin(2*pi*teta/beta_i(j)));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta; 
            
        case 11
            
            y=L(j)*(cos(0.5*pi*teta/beta_i(j)));
            dy=-((pi*L(j))/(2*beta_i(j)))*sin(0.5*pi*teta/beta_i(j));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);

    
            T(n)=teta; 
            
        case 12
            
            y=L(j)*(1-sin(pi*0.5*teta/beta_i(j)));
            dy=-0.5*pi*(L(j)/beta_i(j))*cos(0.5*pi*teta/beta_i(j));
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
            
        case 13
            
            y=L(j)*(1-(teta/beta_i(j))+(pi^-1)*sin(pi*(teta/beta_i(j))));
            dy=-(L(j)/beta_i(j))*(1-cos(pi*(teta/beta_i(j))));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
        
        case 14
            
            y=L(j)*(1-(teta/beta_i(j))-(pi^-1)*sin(pi*(teta/beta_i(j))));
            dy=-(L(j)/beta_i(j))*(1+cos(pi*(teta/beta_i(j))));
    
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
            
        case 15
            
            y=L(j);
            dy=0;
            
            u=(R_0+y)*sin(teta)+dy*cos(teta);
            v=(R_0+y)*cos(teta)-dy*sin(teta);
            
            T(n)=teta;
                   
        end
        
    T(n)=teta;
    Te=[Te; teta+beta_i(j-1)];
    A(n,:,j)=[y,dy];
    Ae = [Ae; y,dy];
    P(n,:,j)=[u,v];
    Pe = [Pe;u,v];
    
    n=n+1;
    teta=teta+step;
    
    end  
    
    figure
    plot(Te,Ae(:,:),'o')
    title('Displacement and acceleration diagrams');
    xlabel('\theta [radians]');
    ylabel('Displacement [m]');
    figure
    title('Plate cam with reciprocating flat-face follower');
    plot(Pe(:,1),Pe(:,2))  
    
    j=j+1; 
      
    end
                   
            
            
            
end
   









