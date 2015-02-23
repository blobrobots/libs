% tiempos de muestreo
dt  = 0.01;    % freq de filtro 100Hz
dtz = 0.1;  % freq de medida 10Hz

rgps  = 0.005; %std of gps measurement
rvgps = 0.05;
rbaro = 0.01; %std of baro measurement
rroc  = 0.05;

%rof=0.15;
%rla=0.1;
% segun carlosIII: 
qpx = 0; qpy = 0; qpz = 0;
qvx = 0; qvy = 0; qvz = 0;
qba = 0.001;

alt_max_rate = 1; %0.1;
roc_max_rate = 1; %0.05;
acc_max_rate = 1;
vel_max_rate = 1;

alt_lpf = 0;
roc_lpf = 0;
acc_lpf = 0;
vel_lpf = 0;

%q=[qpx,qpy,qpz,qvx,qvy,qvz,qba,qba,qba];    %std of process: qpos qvel qacc
%r=[rgps,rgps,rbarot,rvgps,rvgps,rroc];        %std of measurement: qpos qvel qacc

Q = [  (qpx*dt)^2    0     0     0      0      0     0     0     0   % covariance of process
         0    (qpy*dt)^2   0     0      0      0     0     0     0
         0      0   (qpz*dt)^2   0      0      0     0     0     0
         0      0     0   (qvx*dt)^2    0      0     0     0     0
         0      0     0     0    (qvy*dt)^2    0     0     0     0
         0      0     0     0      0    (qvz*dt)^2   0     0     0
         0      0     0     0      0      0   (qba*dt)^2   0     0
         0      0     0     0      0      0     0   (qba*dt)^2   0
         0      0     0     0      0      0     0     0   (qba*dt)^2 ];
     
R = [ (rgps*dtz)^2    0     0     0      0      0  % covariance of measurement
         0   (rgps*dtz)^2   0     0      0      0
         0      0  (rbaro*dtz)^2   0      0      0
         0      0     0  (rvgps*dtz)^2    0      0
         0      0     0     0   (rvgps*dtz)^2    0
         0      0     0     0      0   (rroc*dtz)^2];

% NAV:
% x = [px, py, pz, vx, vy, vz, axb, ayb, azb] = [pos, vel, acc_bias];
% u = [roll, pitch, yaw, ax, ay, az];
% z = [px, py, pz, vx, vy, vz] = [pos,vel];

f = @(x,args)f_nav9(x,args.u,args.dt);              % prediction equation
h = @(x,args)[x(1); x(2); x(3); x(4); x(5); x(6)];  % measurement equation
x = zeros(9,1);          % initial state
N = size(x,1);           % number of states   
P = eye(N);              % initial state covraiance
u=[0 0 0 0 0 -9.8];      % initial input
z = [0;0;0;0;0;0];       % initial measurement

imu_ax = imu(2).data(:,4);
imu_ay = imu(2).data(:,5);
imu_az = imu(2).data(:,6);
imu_roll  = imu(2).data(:,10);
imu_pitch = imu(2).data(:,11);
imu_yaw   = imu(2).data(:,12);
baro_alt = baro(1).data(:,1);
gps_vx = gps(1).data(:,1);
gps_vy = gps(1).data(:,2);
gps_px = gps(1).data(:,3);
gps_py = gps(1).data(:,4);
gps_status = gps(1).data(:,5);
gps_index = gps(1).data(:,6);

time=size(gps_index,1);                    % total dynamic steps (1 per ms)

xV = zeros(N,time);
uV = zeros(6,time);
zV = zeros(6,time);
last_gps_index = 0;
roc=0;
alt=0;
prev_alt=0;
init_altitude = baro_alt(1);

for t=1:time
    if (mod(t,dt*1000) == 0)
        alt = alt + constrain((alt_lpf*alt + (1-alt_lpf)*(baro_alt(t)-init_altitude))-alt, -alt_max_rate, alt_max_rate);
        roc = roc + constrain((roc_lpf*roc + (1-roc_lpf)*(alt-prev_alt)/dt) - roc, -roc_max_rate, roc_max_rate);
        prev_alt = alt;
        
        u(1) = imu_roll(t); % roll
        u(2) = imu_pitch(t); % pitch
        u(3) = imu_yaw(t); % yaw
        
        u(4) = u(4) + constrain(acc_lpf*u(4) + (1-acc_lpf)*imu_ax(t) - u(4), -acc_max_rate, acc_max_rate);
        u(5) = u(5) + constrain(acc_lpf*u(5) + (1-acc_lpf)*imu_ay(t) - u(5), -acc_max_rate, acc_max_rate);
        u(6) = u(6) + constrain(acc_lpf*u(6) + (1-acc_lpf)*imu_az(t) - u(6), -acc_max_rate, acc_max_rate);
        
        if(gps_status(t) < 1) % gps status not OK
            init_altitude = 0.9*init_altitude+0.1*baro.data(t,1);  
            [x, P] = ukf(dt,x,P,f,u,Q,h,z,R);  % ukf predict + measurement update   
        elseif(gps_index(t) ~= last_gps_index)
            last_gps_index = gps_index(t);
            z = [gps_px(t); gps_py(t); -alt; gps_vx(t); gps_vy(t); -roc]; % measurements
            [x, P] = ukf(dt,x,P,f,u,Q,h,z,R);  % ukf predict + measurement update
        else
            [x, P] = ukf(dt,x,P,f,u,Q); % ukf  only predict 
        end
    end
    xV(:,t) = x;                            % save estimate
    uV(:,t) = u;
    zV(:,t) = z;
end

