n = size(traj,1);
t = (0:n-1)*0.08;

traj_c = traj;
traj_c(:,7:9) = traj_c(:,7:9)/180*pi;
angle_ab = [[0,0,0];bias_esti(:,1:3)]/180*pi;
for k=1:n
    Cna = angle2dcm(traj_c(k,7),traj_c(k,8),traj_c(k,9));
    Cab = angle2dcm(angle_ab(k,1),angle_ab(k,2),angle_ab(k,3));
    Cnb = Cab*Cna;
    [r1,r2,r3] = dcm2angle(Cnb);
    traj_c(k,7:9) = [r1,r2,r3]/pi*180;
end

error = nav(1:32:end,:) - traj_c;
error(:,1:2) = error(:,1:2)/180*pi*6378137;
for k=1:n
    if error(k,7)>300
        error(k,7) = error(k,7)-360;
    elseif error(k,7)<-300
        error(k,7) = error(k,7)+360;
    end
    if error(k,9)>300
        error(k,9) = error(k,9)-360;
    elseif error(k,9)<-300
        error(k,9) = error(k,9)+360;
    end
end

figure
subplot(3,3,1)
plot(t, error(:,1))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itL\rm(m)')
grid on

subplot(3,3,4)
plot(t, error(:,2))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\lambda(m)')
grid on

subplot(3,3,7)
plot(t, error(:,3))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\ith\rm(m)')
grid on

subplot(3,3,2)
plot(t, error(:,4))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_n\rm(m/s)')
grid on

subplot(3,3,5)
plot(t, error(:,5))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_e\rm(m/s)')
grid on

subplot(3,3,8)
plot(t, error(:,6))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_d\rm(m/s)')
grid on

subplot(3,3,3)
plot(t, error(:,7))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\psi(\circ)')
grid on

subplot(3,3,6)
plot(t, error(:,8))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\theta(\circ)')
grid on

subplot(3,3,9)
plot(t, error(:,9))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\gamma(\circ)')
grid on

%_________________________________________________________________________%

n = size(bias_esti,1);
t = (1:n)*dt;

figure
subplot(3,3,1)
plot(t, bias_esti(:,1))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,4)
plot(t, bias_esti(:,2))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,7)
plot(t, bias_esti(:,3))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,2)
plot(t, bias_esti(:,4))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,5)
plot(t, bias_esti(:,5))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,8)
plot(t, bias_esti(:,6))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,3)
plot(t, bias_esti(:,7))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,6)
plot(t, bias_esti(:,8))
set(gca, 'xlim', [t(1),t(end)])
grid on
subplot(3,3,9)
plot(t, bias_esti(:,9))
set(gca, 'xlim', [t(1),t(end)])
grid on