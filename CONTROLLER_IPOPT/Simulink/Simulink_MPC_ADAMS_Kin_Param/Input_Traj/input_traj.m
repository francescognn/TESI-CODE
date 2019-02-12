function [N,T,Tsample,t_total,xd,nometraj,initialize_starting_point,x0_val]=input_traj(N,T,Tsample,what,type,init)
% Function that returns the reference trajectory for our controller 
%      together with some parameters necessary for Simulink
%
%'what' is a string that can be: 'base' or 'MM'
%'type' is a string that can be: 
%   - 'line','curve' or 'L' for base motion
%   - 'line','line_orient','sine','sine_orient' or 'NO' for MM motion

% RESTARTING FROM P0??
initialize_starting_point = init;

q0 = [-0.0000   -0.0000   -0.0001   -5.2235   -1.0817   -2.0377    0.3426    0.6332    1.3451];
p0 = FK(q0);
% p0 = [0.6791;-0.1069;1.4720];

x0_val = [q0,p0',zeros(1,6)]';
% if isstring(type)==false
%     error('the input trajectory type is not a string');
% elseif isstring(what)==false
%     error('choose between <<base>> and <<MM>> to control');
% end

switch what
    case 'base'
        switch type 
            case 'line'
                t_total = 35;
                tt = 0:T:t_total;
                v = 0.3;
                xd = [v*tt;zeros(2,length(tt))];
                xd = [xd;q0(4:end)'*ones(1,length(tt));zeros(9,length(tt))];
                nometraj='Base_line';
            case 'curve'
                t_total = 30;
                radius = 3;
                thf = pi/2;
                tt = 0:T:t_total;
                th=linspace(0,thf,length(tt));
                xd=[radius*cos(th);-radius*sin(th)+radius;th];
                xd = [xd;q0(4:end)'*ones(1,length(tt));zeros(9,length(tt))];
                nometraj='Base_curve';
            case 'L'
                t_total=30;
                tt = 0:T:t_total;
                t1=7;t2=10;
                th=linspace(0,pi/2,length(t1:T:t2)); j=1;
                xd=[0;0;0];
                v=0.3;
                for ii =1:length(tt)
                    if tt(ii)<=t1
                        xd = [xd, xd(:,end)+[v*T;0;0]];
                    elseif tt(ii)<=t2
                        radius = 0.5;
                        v_c=(pi/2)/(t2-t1)*radius;
                        theta=th(j);j=j+1;
                        xd=[xd,xd(:,end)+[v_c*cos(theta)*T;v_c*sin(theta)*T;theta]];
                    else 
                        xd = [xd, xd(:,end)+[0;v*T;0]];
                    end
                end
                xd = [xd;q0(4:end)'*ones(1,length(tt));zeros(9,length(tt))];
                nometraj='Base_L';
            otherwise
                error('invalid trajectory type');
        end
        plot(xd(1,:),xd(2,:),'-o','MarkerSize',2)
        grid on; axis equal; title('Trajectory');
    case 'MM'
        switch type 
            case 'line'
                t_total=30; tt=0:T:t_total;
                dist = 1.2;
                v=dist/t_total; 
                xd=[p0(1)+v*tt;p0(2)+v*tt;p0(3)+0.1/t_total*tt];
                xd = [zeros(9,length(tt));xd;zeros(6,length(tt))];
                nometraj='MM_line';
            case 'line_orient'
                t_total=30; tt=0:T:t_total;
                dist = 1.2;
                v=dist/t_total; 
                z_axis = [0;-1;0]; x_axis=[0;0;1];
                xd=[p0(1)+v*tt;p0(2)+v*tt;p0(3)+0.1/t_total*tt];
                xd = [zeros(9,length(tt));xd;[x_axis;z_axis]*ones(1,length(tt))];
                nometraj='MM_lineOrient';
            case 'sine'
                t_total=60; tt=0:T:t_total;
                dist = 1.2;
                noscillazioni = 2;
                om=(noscillazioni*2*pi)/t_total;
                v=dist/t_total; 
                xd=[p0(1)+v*tt;p0(2)+v*tt;p0(3)-0.2*sin(om*tt)];
                xd = [zeros(9,length(tt));xd;zeros(6,length(tt))];
                nometraj='MM_sine';
            case 'sine_orient'
                t_total=60; tt=0:T:t_total;
                dist = 1.2;
                z_axis = [0;1;0]; x_axis=[1;0;0];
                noscillazioni = 2;
                om=(noscillazioni*2*pi)/t_total;
                v=dist/t_total; 
                xd=[p0(1)+v*tt;p0(2)+v*tt;p0(3)-0.2*sin(om*tt)];
                xd = [zeros(9,length(tt));xd;[x_axis;z_axis]*ones(1,length(tt))];
                nometraj='MM_sineOrient';
            case 'NO'
                load('../Data_saved/datax_y.mat')
                xx=xx.*0.6;
                yy=yy.*0.6;
                xd=[xx+p0(1)-xx(1);p0(2).*ones(size(xx));yy+p0(3)-yy(1)];
                z_axis = [0;-1;0]; x_axis=[0;0;1];
                xd = [zeros(9,length(xx));xd;[x_axis;z_axis]*ones(1,length(xx))];
                t_total=length(xx)*T;
                nometraj='MM_NO';
            otherwise
                error('invalid trajectory type');
        end
        plot3(xd(10,:),xd(11,:),xd(12,:),'-o','MarkerSize',2);
        grid on; axis equal; title('Trajectory');
    otherwise
        error('invalid trajectory type');
end

end



 
 
 