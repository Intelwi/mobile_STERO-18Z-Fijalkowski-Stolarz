bag = rosbag('zad4_lab4.bag');

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
xlim([-0.1 1.1]);
ylim([-0.1 1.4]);
title(['Wykresy testów']);
hold off;

%/elektron/mobile_base_controller/odom
bSel = select(bag,"Topic",'/elektron/mobile_base_controller/odom');
msgStructs = readMessages(bSel);
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);

%figure(2)
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
legend('/gazeboOdom','/elektron/odom','/pose2D')
hold off;
print ('tests', '-dpng', '-r400')

%wykres porownanwczy
figure(2)
X=[0 1 1 0 0]
Y=[0 0 1 1 0]
plot(X,Y)
xlim([-0.1 1.1]);
ylim([-0.1 1.4]);
title(['Wykres porównawczy']);

print ('compare_fig', '-dpng', '-r400')