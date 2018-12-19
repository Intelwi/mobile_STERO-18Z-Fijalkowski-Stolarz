bag = rosbag('zad4_lab4_right.bag');

%gazebo_odom
bSel = select(bag,"Topic",'/gazebo_odom');
msgStructs = readMessages(bSel);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);

figure(1)   
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
title(['Wykres porównawczy']);
hold off;
print ('compare_fig', '-dpng', '-r400')

%/elektron/mobile_base_controller/odom
bSel = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs = readMessages(bSel);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);

figure(2)
hold on;
plot(xPoints,yPoints)
hold off;

%/pose2D
bSel = select(bag,"Topic",'/pose2D');
msgStructs = readMessages(bSel);
xPoints = cellfun(@(m) double(m.X),msgStructs);
yPoints = cellfun(@(m) double(m.Y),msgStructs);

%figure(3)
hold on;
plot(xPoints,yPoints)
xlabel('x');
ylabel('y');
xlim([-0.1 1.1]);
ylim([-1.2 0.3]);
title(['Wykresy testów']);
legend('/elektron/odom','/pose2D')
hold off;
print ('tests', '-dpng', '-r400')