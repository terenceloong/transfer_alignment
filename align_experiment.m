%transfer alignment

earth_constant;
dt = 0.0025;

%--navigation initial value--%
p = traj(1,1:3)'; %deg
v = traj(1,4:6)'; %m/s
att = traj(1,7:9)'; %deg
pva0 = [p;v;att]'; %record initial value

%--step--%
n = size(imu,1)-1; %the number of inertial solving

%--filter--%
N_a = 18;
X_a = zeros(N_a,1);
switch N_a
    case 15
        P_a = diag([[1,1,1]*(2/180*pi), [1,1,1]*(2/180*pi), [1,1,1]*2, [1/a,1/a,1]*5, [1,1,1]*(0.1/180*pi)].^2);
        Q_a = diag([[1,1,1]*(0.2/180*pi), [1,1,1]*(0.05/180*pi), [1,1,1]*0.02,...
                    [1/a,1/a,1]*0.05, [1,1,1]*(10/3600/180*pi)].^2) *dt^2;
    case 18
        P_a = diag([[1,1,1]*(2/180*pi), [1,1,1]*(2/180*pi), [1,1,1]*2, [1/a,1/a,1]*5, [1,1,1]*(0.1/180*pi), [1,1,1]*0.05].^2);
        Q_a = diag([[1,1,1]*(0.2/180*pi), [1,1,1]*(0.05/180*pi), [1,1,1]*0.02,...
                    [1/a,1/a,1]*0.05, [1,1,1]*(10/3600/180*pi), [1,1,1]*0.001].^2) *dt^2;
end
R_a = diag([[1,1,1]*(0.05/180*pi), [1,1,1]*0.05, [1/a,1/a,1]*5].^2);

bias_esti = zeros(n,9);
filter_P_a = zeros(n,N_a); %state variable standard deviation

%*************************************************************************%
%--store--%
nav = zeros(n,9); %[lat, lon, h, vn, ve, vd, yaw, pitch, roll]

%--initialize--%
avp = nav_init(p, v, att);
dgyro = [0;0;0]; %gyro compensation
dacc = [0;0;0]; %accelerometer compensation
qab = [1,0,0,0];

for k=1:n
    t = k*dt;
    
    %--IMU data--%
    gyro0 = (imu(k  , 1:3)'-dgyro) /180*pi;
    gyro2 = (imu(k+1, 1:3)'-dgyro) /180*pi;
    gyro1 = (gyro0+gyro2)/2;
    acc0  = (imu(k  , 4:6)'-dacc);
    acc2  = (imu(k+1, 4:6)'-dacc);
    acc1  = (acc0+acc2)/2;
    
    %--inertial navigation solution--%
    avp = ins_solve_3(avp, dt, [gyro0;acc0],[gyro1;acc1],[gyro2;acc2]);
%=========================================================================%
if t<88
    if mod(k,32)~=0
        %---------time update----------%
        Phi_a = state_align(avp, acc1, dt, N_a);
        [X_a, P_a] = kalman_filter(Phi_a, X_a, P_a, Q_a);
    else
        %---------measure update----------%
        Phi_a = state_align(avp, acc1, dt, N_a);
        [H_a, Z_a] = measure_align(avp, traj(k/32+1,:), qab, N_a);
        [X_a, P_a, E_a, Xc_a] = kalman_filter(Phi_a, X_a, P_a, Q_a, H_a, Z_a, R_a);
        
        %---------adjust----------%
        if norm(X_a(1:3))>0
            phi = norm(X_a(1:3));
            qc = [cos(phi/2), X_a(1:3)'/phi*sin(phi/2)];
            avp(1:4) = quatmultiply(qc, avp(1:4)')';
        end
        avp(5:10) = avp(5:10) - X_a(7:12);
        if norm(X_a(4:6))>0
            phi = norm(X_a(4:6));
            qc = [cos(phi/2), X_a(4:6)'/phi*sin(phi/2)];
            qab = quatmultiply(qab, qc);
        end
        dgyro = dgyro + X_a(13:15)/pi*180;
        if N_a==18
            dacc = dacc + X_a(16:18);
        end
        X_a = zeros(N_a,1);
    end
end
    %---------store filter----------%
    [r1,r2,r3] = quat2angle(qab);
    bias_esti(k,1:3) = [r1,r2,r3] /pi*180;
    bias_esti(k,4:6) = dgyro';
    bias_esti(k,7:9) = dacc';
    filter_P_a(k,:) = sqrt(diag(P_a))';

%=========================================================================%
    %--store--%
    nav(k,1:2) = avp(8:9)' /pi*180; %deg
    nav(k,3) = avp(10); %m
    nav(k,4:6) = avp(5:7)'; %m/s
    [r1,r2,r3] = quat2angle(avp(1:4)');
    nav(k,7:9) = [r1,r2,r3] /pi*180; %deg
end
nav = [pva0; nav];

plot_naverror_e;

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
function avp = nav_init(p, v, att)
    p(1:2) = p(1:2)/180*pi;
    att = att/180*pi;
    q = angle2quat(att(1), att(2), att(3));
    avp = [q'; v; p];
    %avp = [q1; q2; q3; q4; vn; ve; vd; lat; lon; h];
    %                           m/s       rad     m
end

function avp = ins_solve_3(avp, dt, imu0, imu1, imu2)
    global a f w
    q = avp(1:4);
    v = avp(5:7);
    v0 = v;
    lat = avp(8);
    lon = avp(9);
    h = avp(10);
    Rm = (1-f)^2*a / (1-(2-f)*f*sin(lat)^2)^1.5;
    Rn =         a / (1-(2-f)*f*sin(lat)^2)^0.5;
    Cnb = quat2dcm(q');
    wien = [w*cos(lat); 0; -w*sin(lat)];
    wenn = [v(2)/(Rn+h); -v(1)/(Rm+h); -v(2)/(Rn+h)*tan(lat)];
    winb = Cnb*(wien+wenn);  
    wnbb0 = imu0(1:3) - winb;
    wnbb1 = imu1(1:3) - winb;
    wnbb2 = imu2(1:3) - winb;
    acc0 = imu0(4:6);
    acc1 = imu1(4:6);
    acc2 = imu2(4:6);
    dtheta1 = (wnbb0+wnbb1)*dt/4;
    dtheta2 = (wnbb1+wnbb2)*dt/4;
    dv1 = (acc0+acc1)*dt/4;
    dv2 = (acc1+acc2)*dt/4;
    q = RK4(@fun_dq, q, dt, wnbb0,wnbb1,wnbb2);
    q = quatnormalize(q')';
    dvc = 0.5*cross(dtheta1,dv1) + 7/6*cross(dtheta1,dv2) - 1/6*cross(dtheta2,dv1) + 0.5*cross(dtheta2,dv2);
    v = v + Cnb'*(dv1+dv2+dvc) - dt*cross((2*wien+wenn),v) + dt*[0;0;gravity(lat,h)];
    lat = lat + dt*(v0(1)+v(1))/2/(Rm+h);
    lon = lon + dt*(v0(2)+v(2))/2/(Rn+h)*sec(lat);
    h = h - dt*(v0(3)+v(3))/2;
    avp = [q; v ; lat; lon; h];
end

function dq = fun_dq(q, w)
    dq = 0.5*[ 0,   -w(1), -w(2), -w(3);
              w(1),   0,    w(3), -w(2);
              w(2), -w(3),   0,    w(1);
              w(3),  w(2), -w(1),   0 ]*q;
end

function Phi = state_align(avp, fb, dt, n)
    global a f
    lat = avp(8);
    h = avp(10);
    Rm = (1-f)^2*a / (1-(2-f)*f*sin(lat)^2)^1.5;
    Rn =         a / (1-(2-f)*f*sin(lat)^2)^0.5;
    Cbn = quat2dcm(avp(1:4)')';
    fn = antisym(Cbn*fb);
    A = zeros(n);
    A(7:9,1:3) = fn;
    A(10:12,7:9) = diag([1/(Rm+h), sec(lat)/(Rn+h), -1]);
    if n ==15 %3-axis gyroscope bias
        A(1:3,13:15) = -Cbn;
    elseif n == 16 %3-axis gyroscope bias + z-axis accelerometer bias
        A(1:3,13:15) = -Cbn;
        A(7:9,16) = Cbn(:,3);
    elseif n ==18 %3-axis gyroscope bias + 3-axis accelerometer bias
        A(1:3,13:15) = -Cbn;
        A(7:9,16:18) = Cbn;
    end
    Phi = eye(n)+A*dt+(A*dt)^2/2;
end

function [H, Z] = measure_align(avp, traj, qab, n) 
    qna = angle2quat(traj(7)/180*pi, traj(8)/180*pi, traj(9)/180*pi);
    qna = quatmultiply(qna,qab);
    Cna = quat2dcm(qna);
    Cbn = quat2dcm(avp(1:4)')';
    C = Cbn*Cna;
    Z = [C(2,3);C(3,1);C(1,2); avp(5:7)-traj(4:6)'; (avp(8:9)-traj(1:2)'/180*pi);avp(10)-traj(3)];
    H = zeros(9,n);
    H(1:3,1:3) = eye(3);
    H(4:6,7:9) = eye(3);
    H(7:9,10:12) = eye(3);
    H(1,4) = Cbn(2,3)*Cna(2,3) - Cbn(2,2)*Cna(3,3);
    H(1,5) = Cbn(2,1)*Cna(3,3) - Cbn(2,3)*Cna(1,3);
    H(1,6) = Cbn(2,2)*Cna(1,3) - Cbn(2,1)*Cna(2,3);
    H(2,4) = Cbn(3,3)*Cna(2,1) - Cbn(3,2)*Cna(3,1);
    H(2,5) = Cbn(3,1)*Cna(3,1) - Cbn(3,3)*Cna(1,1);
    H(2,6) = Cbn(3,2)*Cna(1,1) - Cbn(3,1)*Cna(2,1);
    H(3,4) = Cbn(1,3)*Cna(2,2) - Cbn(1,2)*Cna(3,2);
    H(3,5) = Cbn(1,1)*Cna(3,2) - Cbn(1,3)*Cna(1,2);
    H(3,6) = Cbn(1,2)*Cna(1,2) - Cbn(1,1)*Cna(2,2);
end