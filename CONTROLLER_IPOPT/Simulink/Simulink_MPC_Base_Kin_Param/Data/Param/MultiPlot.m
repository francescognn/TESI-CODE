clear all
close all
clc

Nvect=[10 15 20 25];
Distvect=[0 1 3];
col=[0 1 1; 0 0.5 1; 0 0 1];

load('MPC_kin_Param_N=10_trajCurves_r=0.6_dist=0.mat')
clearvars -EXCEPT xd Nvect Distvect col

for ii=1:length(Nvect)
    figure(1)
    subplot(1,4,ii)
    plot(xd(1,:),xd(2,:),'r','LineWidth',3)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        plot(x,y,'.-','Color',col(j,:),'LineWidth',1)
        set(gcf,'color','white')
        grid on
        clearvars -EXCEPT xd Nvect Distvect ii j col

    end
    legend('REFERENCE','no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m'])
    axis equal
    hold off
end
mean_error=zeros(length(Nvect),length(Distvect));
mean_telaps=zeros(length(Nvect),length(Distvect));
for ii=1:length(Nvect)
    figure(2)
    subplot(2,2,ii)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';
        CartError=sqrt(errore(:,1).^2+errore(:,2).^2);
        mean_error(ii,j)=mean(CartError);
        set(gcf,'color','white')
        plot(tt,CartError,'-','Color',col(j,:),'LineWidth',1)
        ylabel('Cartesian Error')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m'])
    hold off
    
    figure(3)
    subplot(2,2,ii)
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        set(gcf,'color','white')
        plot(tt,T_elapsed_vect(1:length(tt)),'-','Color',col(j,:),'LineWidth',1)
        mean_telaps(ii,j)=mean(T_elapsed_vect(1:length(tt)));
        ylabel('T Elapsed')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m'])
    hold off
end

figure(4)
title('r=0.6m')
for j = 1:length(Distvect)
    subplot(2,1,1)
    plot(Nvect,mean_error(:,j),'-o','Color',col(j,:),'LineWidth',1)
    hold on
    title('r=0.6m');
    ylabel('Mean Cartesian error')
    xlabel('Prediction horizon')
    subplot(2,1,2)
    plot(Nvect,mean_telaps(:,j),'-o','Color',col(j,:),'LineWidth',1)
    title('r=0.6m');
    ylabel('Mean time elapsed')
    xlabel('Prediction horizon')
    hold on
end
legend('no disturb','small disturb','big disturb','very big disturb')
errors_poly3=mean(mean_error,2);
telaps_poly3=mean(mean_telaps,2);
%%
load('MPC_kin_Param_N=25_trajCurves_r=0.3_dist=0.mat')
clearvars -EXCEPT xd Nvect Distvect errors_poly3 telaps_poly3 col

for ii=1:length(Nvect)
    figure(5)
    subplot(1,4,ii)
    plot(xd(1,:),xd(2,:),'r','LineWidth',2)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.3_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        plot(x,y,'.-','Color',col(j,:),'LineWidth',1)
        set(gcf,'color','white')
        grid on
        clearvars -EXCEPT xd Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('REFERENCE','no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),'r=0.3m'])
%     axis equal
    hold off
end
mean_error=zeros(length(Nvect),length(Distvect));
mean_telaps=zeros(length(Nvect),length(Distvect));
for ii=1:length(Nvect)
    figure(6)
    subplot(2,2,ii)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.3_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';
        CartError=sqrt(errore(:,1).^2+errore(:,2).^2);
        mean_error(ii,j)=mean(CartError);
        set(gcf,'color','white')
        plot(tt,CartError,'-','Color',col(j,:),'LineWidth',1)
        ylabel('Cartesian Error')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.3m'])
    hold off
    
    figure(7)
    subplot(2,2,ii)
    for j=1:length(Distvect)
        load(['MPC_kin_Param_N=',num2str(Nvect(ii)),'_trajCurves_r=0.3_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        set(gcf,'color','white')
        plot(tt,T_elapsed_vect(1:length(tt)),'-','Color',col(j,:),'LineWidth',1)
        mean_telaps(ii,j)=mean(T_elapsed_vect(1:length(tt)));
        ylabel('T Elapsed')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.3m'])
    hold off
end

figure(8)

for j = 1:length(Distvect)
    subplot(2,1,1)
    plot(Nvect,mean_error(:,j),'-o','Color',col(j,:),'LineWidth',1)
    hold on
    title('r=0.3m');
    ylabel('Mean Cartesian error')
    xlabel('Prediction horizon')
    subplot(2,1,2)
    plot(Nvect,mean_telaps(:,j),'-o','Color',col(j,:),'LineWidth',1)
    ylabel('Mean time elapsed')
    xlabel('Prediction horizon')
    hold on
end
legend('no disturb','small disturb','big disturb','very big disturb')

%%
load('MPC_kin_Param2_N=10_trajCurves_r=0.6_dist=0.mat')
clearvars -EXCEPT xd Nvect Distvect errors_poly3 telaps_poly3 col

for ii=1:length(Nvect)
    figure(9)
    subplot(1,4,ii)
    plot(xd(1,:),xd(2,:),'r','LineWidth',2)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param2_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        plot(x,y,'.-','Color',col(j,:),'LineWidth',1)
        set(gcf,'color','white')
        grid on
        clearvars -EXCEPT xd Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('REFERENCE','no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m  POLY5'])
    axis equal
    hold off
end
mean_error=zeros(length(Nvect),length(Distvect));
mean_telaps=zeros(length(Nvect),length(Distvect));
for ii=1:length(Nvect)
    figure(10)
    subplot(2,2,ii)
    hold on
    for j=1:length(Distvect)
        load(['MPC_kin_Param2_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        x=X(1,:);y=X(2,:);th=X(3,:);
        errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';
        CartError=sqrt(errore(:,1).^2+errore(:,2).^2);
        mean_error(ii,j)=mean(CartError);
        set(gcf,'color','white')
        plot(tt,CartError,'-','Color',col(j,:),'LineWidth',1)
        ylabel('Cartesian Error')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m  POLY5'])
    hold off
    
    figure(11)
    subplot(2,2,ii)
    for j=1:length(Distvect)
        load(['MPC_kin_Param2_N=',num2str(Nvect(ii)),'_trajCurves_r=0.6_dist=',num2str(Distvect(j)),'.mat'])
        
        for i=1:size(simout.Data,3)
            i=real(i);
            X(:,i)=simout.Data(1,:,i);   
        end
        if size(X,2)>=size(xd,2)
            X=X(:,1:size(xd,2));
        else
            xd=xd(:,1:size(X,2));
        end
        set(gcf,'color','white')
        plot(tt,T_elapsed_vect(1:length(tt)),'.-','Color',col(j,:),'LineWidth',1)
        mean_telaps(ii,j)=mean(T_elapsed_vect(1:length(tt)));
        ylabel('T Elapsed')
        hold on
        grid on
        clearvars -EXCEPT mean_error mean_telaps Nvect Distvect ii j errors_poly3 telaps_poly3 col

    end
    legend('no disturb','small disturb','big disturb','very big disturb')
    title(['N=',num2str(Nvect(ii)),' r=0.6m  POLY5'])
    hold off
end


figure(12)

for j = 1:length(Distvect)
    subplot(2,1,1)
    plot(Nvect,mean_error(:,j),'-o','Color',col(j,:),'LineWidth',1)
    hold on
    title('r=0.6m  POLY5');
    ylabel('Mean Cartesian error')
    xlabel('Prediction horizon')
    subplot(2,1,2)
    plot(Nvect,mean_telaps(:,j),'-o','Color',col(j,:),'LineWidth',1)
    title('r=0.6m  POLY5');
    ylabel('Mean time elapsed')
    xlabel('Prediction horizon')
    hold on
end
legend('no disturb','small disturb','big disturb','very big disturb')

telaps_poly5=mean(mean_telaps,2);
errors_poly5=mean(mean_error,2);

figure;
subplot(2,1,1)
plot(Nvect,errors_poly3,'r-o','LineWidth',1)
hold on
plot(Nvect,errors_poly5,'b-*','LineWidth',1)
title('mean Cartesian Errors');
xlabel('Prediction horizon')
legend('POLY3','POLY5')
subplot(2,1,2)
plot(Nvect,telaps_poly3,'r-o','LineWidth',1)
hold on
plot(Nvect,telaps_poly5,'b-*','LineWidth',1)
title('mean Times elapsed');
xlabel('Prediction horizon')
legend('POLY3','POLY5')


