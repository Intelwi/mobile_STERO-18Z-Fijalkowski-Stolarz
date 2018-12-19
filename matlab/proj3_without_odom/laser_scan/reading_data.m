%błąd kwadratu
% bag = rosbag('proj3_laser_square_error.bag');
% 
% %błąd
% bSel1 = select(bag,"Topic",'/error');
% msgStructs1 = readMessages(bSel1);
% x_error = cellfun(@(m) double(m.X),msgStructs1);
% y_error = cellfun(@(m) double(m.Y),msgStructs1);
% theta_error = cellfun(@(m) double(m.Theta),msgStructs1);
% 
% error = (x_error.^2 + y_error.^2).^(0.5)
% 
% figure(1)
% plot(error)
% xlim([0 length(error)]);
% xlabel('time');
% ylabel('path error');
% title(['Błąd kwadratu laser'])
% print ('square_path_error', '-dpng', '-r400')
%         
% figure(2)   
% hold on;
% title(['Błąd kwadratu laser'])
% plot(theta_error);
% xlabel('time');
% ylabel('theta error');
% print ('square_theta_error', '-dpng', '-r400')
% 
% %ścieżka
% bSel2 = select(bag,"Topic",'/gazebo_odom');
% msgStructs2 = readMessages(bSel2);
% x = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
% y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);
% figure(3)
% plot(x,y)
% title(['Trasa kwadrat']);
% xlabel('x');
% ylabel('y');
% print ('square_path', '-dpng', '-r400')

%błąd ruchu naprzód i spwerotem
bag = rosbag('proj3_laser_slide_error.bag');

%błąd
bSel1 = select(bag,"Topic",'/error');
msgStructs1 = readMessages(bSel1);
x_error = cellfun(@(m) double(m.X),msgStructs1);
y_error = cellfun(@(m) double(m.Y),msgStructs1);
theta_error = cellfun(@(m) double(m.Theta),msgStructs1);

error = (x_error.^2 + y_error.^2).^(0.5)

figure(4)
plot(error)
xlim([0 length(error)]);
xlabel('time');
ylabel('path error');
title(['Błąd slide laser'])
print ('slide_path_error', '-dpng', '-r400')
     
figure(5)   
hold on;
title(['Błąd slide laser'])
plot(theta_error);
xlabel('time');
ylabel('theta error');
print ('slide_theta_error', '-dpng', '-r400')


%ścieżka
bSel2 = select(bag,"Topic",'/gazebo_odom');
msgStructs2 = readMessages(bSel2);
x = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);
figure(6)
plot(x,y)
title(['Trasa slide']);
xlabel('x');
ylabel('y');
print ('slide_path', '-dpng', '-r400')

%błąd ruchu naprzód i spowrotem
bag = rosbag('proj3_laser_turn_error.bag');

%błąd
bSel1 = select(bag,"Topic",'/error');
msgStructs1 = readMessages(bSel1);
x_error = cellfun(@(m) double(m.X),msgStructs1);
y_error = cellfun(@(m) double(m.Y),msgStructs1);
theta_error = cellfun(@(m) double(m.Theta),msgStructs1);

error = (x_error.^2 + y_error.^2).^(0.5)

figure(7)
plot(error)
xlim([0 length(error)]);
xlabel('time');
ylabel('path error');
title(['Błąd obrotu laser'])
print ('rotate_path_error', '-dpng', '-r400')
  

figure(8)   
hold on;
title(['Błąd obrotu laser'])
plot(theta_error);
xlabel('time');
ylabel('theta error');
print ('rotate_theta_error', '-dpng', '-r400')


%ścieżka
bSel2 = select(bag,"Topic",'/gazebo_odom');
msgStructs2 = readMessages(bSel2);
x = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs2);
y = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs2);
figure(9)
plot(x,y)
title(['Trasa obrotu']);
xlabel('x');
ylabel('y');
print ('rotate_path', '-dpng', '-r400')

%print ('tests', '-dpng', '-r400')
%

%print ('tests', '-dpng', '-r400')
%