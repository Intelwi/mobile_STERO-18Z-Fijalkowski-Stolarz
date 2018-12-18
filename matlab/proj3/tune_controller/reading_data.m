%błąd kwadratu
%

bag = rosbag('proj3_tune_drive_square_error.bag');

%gazebo_odom
bSel1 = select(bag,"Topic",'/error');
msgStructs1 = readMessages(bSel1);
x_error = cellfun(@(m) double(m.X),msgStructs1);
y_error = cellfun(@(m) double(m.Y),msgStructs1);
theta_error = cellfun(@(m) double(m.Theta),msgStructs1);

error = (x_error.^2 + y_error.^2).^(0.5)

figure(1)
plot(error)
xlim([0 length(error)]);
title(['Błąd kwadratu tune'])
        
figure(2)   
hold on;
plot(theta_error)
xlabel('x');
ylabel('y');
%xlim([-1.1 0.1]);
%ylim([-0.1 1.1]);
%title(['Trajektoria']);
%hold off;
%gazebo_odom
bSel2 = select(bag,"Topic",'/gazebo_odom');
msgStructs2 = readMessages(bSel2);
x = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);
figure(3)
plot(x,y)
title(['Trasa'])
        
%print ('tests', '-dpng', '-r400')
%

%print ('tests', '-dpng', '-r400')
%
