n = size(bias_esti,1);
t = (1:n)*dt;

figure
plot(t, drift_g(2:end,1)+gyro_bias(1)/pi*180)
hold on
plot(t, bias_esti(:,4))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_x\rm(\circ)')
grid on

figure
plot(t, drift_g(2:end,2)+gyro_bias(2)/pi*180)
hold on
plot(t, bias_esti(:,5))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_y\rm(\circ)')
grid on

figure
plot(t, drift_g(2:end,3)+gyro_bias(3)/pi*180)
hold on
plot(t, bias_esti(:,6))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_z\rm(\circ)')
grid on