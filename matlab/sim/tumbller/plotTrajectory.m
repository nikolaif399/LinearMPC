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
    
    height = 0.16;
    
    hold on
    plot(qCur(1)+[0,height*sin(qCur(3))],[0,height*cos(qCur(3))],'r')
    plot([-2.2 2.2],[0 0],'k')
    xlim([-0.5 0.5])
    ylim([-0.2 0.8])
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


