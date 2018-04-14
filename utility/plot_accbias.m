n = size(bias_esti,1);
t = (1:n)*dt;

drift_a = zeros(n+1,3);

figure
plot(t, drift_a(2:end,1)+acc_bias(1))
hold on
plot(t, bias_esti(:,7))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\nabla\it_x\rm(m/s^2)')
grid on

figure
plot(t, drift_a(2:end,2)+acc_bias(2))
hold on
plot(t, bias_esti(:,8))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\nabla\it_y\rm(m/s^2)')
grid on

figure
plot(t, drift_a(2:end,3)+acc_bias(3))
hold on
plot(t, bias_esti(:,9))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\nabla\it_z\rm(m/s^2)')
grid on