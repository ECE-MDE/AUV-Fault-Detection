figure
hold on
grid on
plot3(x_hat_tot(7,:),x_hat_tot(8,:),x_hat_tot(9,:),'LineWidth',1.5, 'Color', 'r')
legend('estimate')
title('Vehicle Trajectory in North-East-Down Coordinate System')
xlabel('north (m)')
ylabel('east (m)')
zlabel('depth (m)')
hold off

figure
hold on
grid on
plot3(x_tot(7,:),x_tot(8,:),x_tot(9,:),'LineWidth',1.5, 'Color', 'b')
legend('ground truth')
title('Vehicle Trajectory in North-East-Down Coordinate System')
xlabel('north (m)')
ylabel('east (m)')
zlabel('depth (m)')
hold off