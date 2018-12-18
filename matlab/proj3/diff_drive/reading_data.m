bag = rosbag('proj3_diff_drive_square.bag');

%gazebo_odom
bSel = select(bag,"Topic",'/gazebo_odom');
msgStructs = readMessages(bSel);
xPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPointsGazebo = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);


%gazebo_odom
bSel = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs = readMessages(bSel);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);


figure(1)   
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
xlim([-1.1 0.1]);
ylim([-0.1 1.1]);
title(['Wykresy testów']);
hold off;

%print ('tests', '-dpng', '-r400')
