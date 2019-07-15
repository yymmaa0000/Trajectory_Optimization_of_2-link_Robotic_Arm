function animate(x, dt, L1, L2,m1,m2,mo,cost,obj_fun,save_video)
% x: collection of state vectors at each time step
% dt: time step
% L1 - mo: paramter of the system
% cost = final value of the cost function
% mode = a string describing the objective function
% save_video ~= 0 means saving the animation into a video
% save_video == 0 means not saving the animation
% default for save_video is 0

if nargin > 9
  bRecord = save_video;
else
  bRecord = 0;
end

if bRecord
    % Define video recording parameters
    Filename = sprintf("%d_%d_%d_%d_%d_%s",L1,L2,m1,m2,mo,obj_fun);
    v = VideoWriter(Filename, 'MPEG-4');
    myVideo.Quality = 100;
    open(v);
end

% Define axis window
len = L1+L2;
xmin = -len;
xmax = len;
ymin = -len;
ymax = len;
[row,col] = size(x);

Fig = figure('Color', 'w','Position', [200 200 1200 500]);
title_string = sprintf('Mass of arm1 = %0.1fkg, Mass of arm2 = %0.1fkg, Mass of object = %0.1fkg, Cost = %.2f, Opjective function = %s',m1,m2,mo,cost,obj_fun);
suptitle(title_string)

% Create trace of trajectory and particle object
t = 0;
arm1 = [];
arm2 = [];
object = [];

% Set up 2 subplots
subplot(1,2,2)
h2 = animatedline('Color','b','LineStyle', '-', 'LineWidth', 1.5);
h3 = animatedline('Color','r','LineStyle', '-', 'LineWidth', 1.5);
axis([0  dt*row  min(min(x(:,6)),min(x(:,5))) max(max(x(:,6)),max(x(:,5)))])
xlabel('x')
ylabel('y')
xlabel('Time (s)')
ylabel('Torque (N*m)')

subplot(1,2,1)
h1 = animatedline('LineStyle', ':', 'LineWidth', 1.5);
axis equal
axis([xmin xmax ymin ymax])
xlabel('x')
ylabel('y')

% draw
base = [0;0];
width = 0.1/2;
armbase = patch('XData',[base(1) base(1)-width base(1)+width],...
    'YData',[base(2)+width base(2)-width base(2)-width]);
for ii = 1:length(x)
    subplot(1,2,1)
    a = tic;
    
    set(gcf,'DoubleBuffer','on');
    
    q1 = x(ii,1);
    q2 = x(ii,2);
    joint1 = [L1*cos(q1);L1*sin(q1)];
    joint2 = [L1*cos(q1) + L2*cos(q1+q2);L1*sin(q1) + L2*sin(q1+q2)]; 
    center1 = (joint1+base)/2;
    center2 = (joint2+joint1)/2;
    rotate1 = [cos(q1) -sin(q1);sin(q1) cos(q1)];
    rotate2 = [cos(q1+q2) -sin(q1+q2);sin(q1+q2) cos(q1+q2)];
    XY1 = [-L1/2 -L1/2 L1/2 L1/2; width -width -width width];
    XY2 = [-L2/2 -L2/2 L2/2 L2/2; width -width -width width];
    delete(arm1);
    delete(arm2);
    coord2 = rotate2*XY2+center2;
    arm2 = patch('XData',coord2(1,:),'YData',coord2(2,:)','EdgeColor','red','FaceColor','none');
    coord1 = rotate1*XY1+center1;
    arm1 = patch('XData',coord1(1,:),'YData',coord1(2,:)','EdgeColor','blue','FaceColor','none');
    delete(object);
    object = line(joint2(1), joint2(2), 'Color', [1;0;0],'Marker','.', 'MarkerSize', 20);
    addpoints(h1,joint2(1), joint2(2));
    
    subplot(1,2,2)
    addpoints(h2,t, x(ii,5));
    addpoints(h3,t, x(ii,6));
    legend('arm1','arm2')
    t = t+dt;
    
    drawnow limitrate
    
    if bRecord
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
        pause(dt - toc(a)); % waits if drawing frame took less time than anticipated
    end
end

if bRecord
    close(v);
end