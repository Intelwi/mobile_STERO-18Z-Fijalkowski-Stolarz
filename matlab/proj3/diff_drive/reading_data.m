%błąd kwadratu
%
bag = rosbag('proj3_diff_drive_square.bag');

%gazebo_odom
bSel1 = select(bag,"Topic",'/gazebo_odom');
msgStructs1 = readMessages(bSel1);
xPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs1);
yPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs1);

%elektron_odom
bSel2 = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs2 = readMessages(bSel2);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);

error = zeros(length(xPointsGazebo),1);
i=1;
j=1;
while(i<=length(yPointsGazebo) && j<=length(yPoints))
    error(i) = ((xPointsGazebo(i)-xPoints(j))^2 + (yPointsGazebo(i)-yPoints(j))^2)^(0.5);
    i=i+1;
    j=j+2;
end
figure(1)
plot(error)
xlim([0 length(xPointsGazebo)]);
title(['Błąd kwadratu diffdrive'])
    
    
figure(2)   
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
xlim([-1.1 0.1]);
ylim([-0.1 1.1]);
title(['Trajektoria']);
hold off;

%print ('tests', '-dpng', '-r400')
%


%błąd jazdy tam i spowrotem
%
bag = rosbag('proj3_diff_drive_slide.bag');

%gazebo_odom
bSel1 = select(bag,"Topic",'/gazebo_odom');
msgStructs1 = readMessages(bSel1);
xPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs1);
yPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs1);

%elektron_odom
bSel2 = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs2 = readMessages(bSel2);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);

error = zeros(length(xPointsGazebo),1);
i=1;
j=1;
while(i<=length(yPointsGazebo) && j<=length(yPoints))
    error(i) = ((xPointsGazebo(i)-xPoints(j))^2 + (yPointsGazebo(i)-yPoints(j))^2)^(0.5);
    i=i+1;
    j=j+2;
end
figure(3)
plot(error)
xlim([0 length(xPointsGazebo)]);
title(['Błąd jazdy tam i sporotem diffdrive'])
    
    
figure(4)   
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
xlim([-1.1 0.1]);
ylim([-0.1 1.1]);
title(['Trajektoria']);
hold off;

%print ('tests', '-dpng', '-r400')
%

%błąd obrotu
%
bag = rosbag('proj3_diff_drive_rotate.bag');

%gazebo_odom
bSel1 = select(bag,"Topic",'/gazebo_odom');
msgStructs1 = readMessages(bSel1);
x_quater_Gazebo = cellfun(@(m) double(m.Pose.Pose.Orientation.X),msgStructs1);
y_quater_Gazebo = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),msgStructs1);
z_quater_Gazebo = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),msgStructs1);
w_quater_Gazebo = cellfun(@(m) double(m.Pose.Pose.Orientation.W),msgStructs1);


[roll,pitch,yaw] = quat2angle([x_quater_Gazebo y_quater_Gazebo z_quater_Gazebo w_quater_Gazebo])



%elektron_odom
bSel2 = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs2 = readMessages(bSel2);
thetaPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);

error = zeros(length(xPointsGazebo),1);
i=1;
j=1;
while(i<=length(yPointsGazebo) && j<=length(yPoints))
    error(i) = abs(thetaPointsGazebo(i)-thetaPoints(j));
    i=i+1;
    j=j+2;
end
figure(5)
plot(error)
xlim([0 length(xPointsGazebo)]);
title(['Błąd obrotu diff_drive'])
    
    
figure(6)   
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
xlim([-1.1 0.1]);
ylim([-0.1 1.1]);
title(['Trajektoria']);
hold off;

%print ('tests', '-dpng', '-r400')
%