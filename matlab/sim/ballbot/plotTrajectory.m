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
    subplot(2,1,1)
    plot(refTraj(1,:),'b')
    hold on;
    plot(optCur(1,:), 'g-')
    plot(qCur(1), '+r', 'MarkerSize', 10)
%     plotQuad(h1,qCur);
%     title('Ballbot Trajectory')
%     
    %axis equal
%     xlim([min(refTraj(1,:))-0.4 max(refTraj(1,:))+0.4])
%     ylim([min(refTraj(2,:))-0.4 max(refTraj(2,:))+0.4])
%     zlim([min(refTraj(3,:))-0.4 max(refTraj(3,:))+0.4])
grid on
%     
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


