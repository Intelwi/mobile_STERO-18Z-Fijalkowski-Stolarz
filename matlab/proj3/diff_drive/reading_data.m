%błąd kwadratu
%

bag = rosbag('proj3_diff_drive_square_error.bag');

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
title(['Błąd kwadratu diffdrive'])
        
%figure(2)   
%hold on;
%plot(xPoints,yPoints)
%xlabel('x');
%ylabel('y');
%xlim([-1.1 0.1]);
%ylim([-0.1 1.1]);
%title(['Trajektoria']);
%hold off;

%print ('tests', '-dpng', '-r400')
%

%print ('tests', '-dpng', '-r400')
%