%błąd kwadratu
bag = rosbag('proj3_diff_drive_ahead_error.bag');

%błąd
bSel1 = select(bag,"Topic",'/error');
msgStructs1 = readMessages(bSel1);
x_error = cellfun(@(m) double(m.X),msgStructs1);
y_error = cellfun(@(m) double(m.Y),msgStructs1);
theta_error = cellfun(@(m) double(m.Theta),msgStructs1);

error = (x_error.^2 + y_error.^2).^(0.5)

figure(1)
plot(error)
xlim([0 length(error)]);
xlabel('time');
ylabel('path error');
title(['Błąd jazdy do przodu diffdrive'])
        
figure(2)   
hold on;
title(['Błąd jazdy do przodu diffdrive'])
plot(theta_error);
xlabel('time');
ylabel('theta error');

bSel2 = select(bag,"Topic",'/gazebo_odom');
msgStructs2 = readMessages(bSel2);
x = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);
figure(3)
plot(x,y)
title(['Trasa kwadrat']);
xlabel('x');
ylabel('y');
        
%print ('diffdrive_test', '-dpng', '-r400')