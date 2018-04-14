n = size(traj,1);
t = (0:n-1)*dt;

figure
plot(traj(:,2),traj(:,1), 'LineWidth',2)
xlabel('\lambda(\circ)')
ylabel('\itL\rm(\circ)')
title('Position')
grid on

figure
subplot(3,1,1)
plot(t, traj(:,4), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\itv_n\rm(m/s)')
title('Velocity')
grid on
subplot(3,1,2)
plot(t, traj(:,5), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\itv_e\rm(m/s)')
grid on
subplot(3,1,3)
plot(t, traj(:,6), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\itv_d\rm(m/s)')
grid on

figure
subplot(3,1,1)
plot(t, traj(:,7), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\psi(\circ)')
title('Attitude')
grid on
subplot(3,1,2)
plot(t, traj(:,8), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\theta(\circ)')
grid on
subplot(3,1,3)
plot(t, traj(:,9), 'LineWidth',2)
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\gamma(\circ)')
grid on