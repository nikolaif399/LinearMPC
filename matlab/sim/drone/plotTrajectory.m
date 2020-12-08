function [] = plotTrajectory(qCache,optCache,refTraj,dt,record)
%PLOTTRAJECTORY 
% qCache Cell array of states
% optCache cell array of optimized trajectory segments

N = size(qCache,2);

if record
    v = VideoWriter([datestr(datetime('now')),'.mp4'],'MPEG-4');
    v.FrameRate = 1/dt;
    open(v)
end

figure(1)
h1 = gca;

for i = 1:N
    tic
    if record
        writeVideo(v,getframe(gcf));
    end
    
    % Collect relevant cached data for this timestep
    optCur = optCache{i};
    qCur = qCache{i};
    
    % Plot
    cla(h1);
    plot3(h1, refTraj(1,:),refTraj(2,:),refTraj(3,:),'b')
    hold on
    plot3(h1, optCur(1,:),optCur(2,:),optCur(3,:), 'g-')
    plotQuad(h1,qCur);
    title('Drone Trajectory')
    
    axis equal
    xlim([min(refTraj(1,:))-0.4 max(refTraj(1,:))+0.4])
    ylim([min(refTraj(2,:))-0.4 max(refTraj(2,:))+0.4])
    zlim([min(refTraj(3,:))-0.4 max(refTraj(3,:))+0.4])
    grid on
    
    % Display in realtime
    tsleep = dt - toc;
    if (tsleep > 0)
        pause(tsleep)
    end
end

if record
    close(v)
end
end


