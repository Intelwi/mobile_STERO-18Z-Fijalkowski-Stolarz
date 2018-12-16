bag = rosbag('session1.bag');
bSel = select(bag);
msgStructs = readMessages(bSel);
msgStructs{1}.Pose.Pose.Position
xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);
plot(xPoints,yPoints)