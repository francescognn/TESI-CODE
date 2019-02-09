function [] = liveplot(state_x)
xd=evalin('base','xd');
fig8=evalin('base','fig8');
robot=evalin('base','robot');

    for j=1:size(state_x,2)

        hold on
        robot.visualize_mm(state_x(1:9,j).',fig8);
        plot3(xd(10,:),xd(11,:),xd(12,:),'r')
        plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')

        xlabel('x')
        ylabel('y')
        zlabel('z')
        grid on

        axis([-1.5+min(xd(10,:)) 1.5+max(xd(10,:)) -1.5+min(xd(11,:)) 1.5+max(xd(11,:)) 0 2])
        view(-10,10);%-10, 20);
        drawnow();
        
    end
end

