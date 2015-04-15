%% ukf_nav9_z42.m
%  author: adrian jimenez gonzalez
%  email:  blobrobotics@gmail.com
%  date:   15-jan-2015
%  brief:  Example of use of the ukf library to estimate position and velocity 

%% save datasets
data_file = 'ukf_nav9_z42.in';
result_file = 'ukf_nav9_z42.out';

%% tiempos de muestreo
dt     = 0.01;    % freq de filtro 100Hz
dtgps  = 0.1;  % freq de medida 10Hz
dtbaro = 0.1;  % freq de medida 10Hz

%% caracterizacion sensores
rgps  = 0.001; %std of gps measurement
rvgps = 0.05;
rbaro = 0.005; %std of baro measurement
rbroc = 0.05;

%rof   = 0.15;
%ralt  = 0.1;
%raroc = 0.1;

qpx = 0; qpy = 0; qpz = 0;
qvx = 0; qvy = 0; qvz = 0;
qba = 0.001;

% q = [qpx,qpy,qpz,qvx,qvy,qvz,qba,qba,qba];  %std of process: qpos qvel qacc
% rgps  = [rgps,rgps,rvgps,rvgps];            %std of gps measurement
% rbaro = [rbaro,rbroc];                      %std of baro measurement
% rof   = [rof,rof];                          %std of OF measurement
% ralt  = [ralt,raroc];                       %std of altimeter measurement

Q = [  (qpx*dt)^2    0     0     0      0      0     0     0     0   % covariance of process
         0    (qpy*dt)^2   0     0      0      0     0     0     0
         0      0   (qpz*dt)^2   0      0      0     0     0     0
         0      0     0   (qvx*dt)^2    0      0     0     0     0
         0      0     0     0    (qvy*dt)^2    0     0     0     0
         0      0     0     0      0    (qvz*dt)^2   0     0     0
         0      0     0     0      0      0   (qba*dt)^2   0     0
         0      0     0     0      0      0     0   (qba*dt)^2   0
         0      0     0     0      0      0     0     0   (qba*dt)^2 ];
     
Rgps = [ (rgps*dtgps)^2    0     0     0    % covariance of gps measurement
            0   (rgps*dtgps)^2   0     0
            0      0  (rvgps*dtgps)^2  0
            0      0     0  (rvgps*dtgps)^2 ];
        
Rbaro = [ (rbaro*dtbaro)^2    0             % covariance of baro measurement
             0   (rbroc*dtbaro)^2   ];

%% input data
imu_ax     = imu(2).data(:,4);
imu_ay     = imu(2).data(:,5);
imu_az     = imu(2).data(:,6);
imu_roll   = imu(2).data(:,10);
imu_pitch  = imu(2).data(:,11);
imu_yaw    = imu(2).data(:,12);
baro_alt   = baro(1).data(:,1);
gps_vx     = gps(1).data(:,1);
gps_vy     = gps(1).data(:,2);
gps_px     = gps(1).data(:,3);
gps_py     = gps(1).data(:,4);
gps_status = gps(1).data(:,5);
gps_index  = gps(1).data(:,6);

%% initial values
% NAV:
% x = [px, py, pz, vx, vy, vz, axb, ayb, azb] = [pos, vel, acc_bias];
% u = [roll, pitch, yaw, ax, ay, az];
% z = [px, py, vx, vy] [pz, vz] = [pos,vel] [z,roc];

f = @(x,args)f_nav9(x,args.u,args.dt);   % prediction equation
h = @(x,args)[x(1); x(2); x(4); x(5);];  % measurement equation
x = zeros(9,1);          % initial state
N = size(x,1);           % number of states   
P = eye(N);              % initial state covraiance
u = [0 0 0 0 0 -9.8];    % initial input
z = [0;0;0;0;];          % initial measurement

time = size(gps_index,1); % total dynamic steps (1 per ms)

xV = zeros(N,time);

last_gps_index = 0;
broc=0;
balt=0;
prev_balt=0;
init_altitude = baro_alt(1);

%% save datasets
df_id = -1; rf_id = -1;
if(exist('data_file') == 1)
   df_id = fopen(data_file,'w');
end
if(exist('result_file') == 1)
   rf_id = fopen(result_file,'w');
end
if(df_id >= 0)
   fprintf(df_id, 'roll pitch yaw ax ay az baro gps_x gps_y gps_vx gps_vy gps_index gps_status (dt=%f)\n',dt);
end
if(rf_id >= 0)
   fprintf(rf_id, 'dt=%f dtgps=%f dtbaro=%f rgps=%f rvgps=%f rbaro=%f rbroc=%f qba=%f\n',dt,dtgps,dtbaro,rgps,rvgps,rbaro,rbroc,qba);
   fprintf(rf_id, 'px py pz vx vy vz abx aby abz\n');
end

%% loop
for t=1:time
    if (mod(t,dt*1000) == 0)
        %% update sensor data
        balt = baro_alt(t)-init_altitude;
        broc = (balt-prev_balt)/dt;
        prev_balt = balt;
        
        u(1) = imu_roll(t); % roll
        u(2) = imu_pitch(t); % pitch
        u(3) = imu_yaw(t); % yaw
        u(4) = imu_ax(t); % roll
        u(5) = imu_ay(t); % pitch
        u(6) = imu_az(t); % yaw
        
        %% prediction step
        [x,P,X,Xs]= ukf_predict(dt,x,P,f,u,Q); % prediction step
        if(gps_status(t) < 1) % gps status not OK
            init_altitude = 0.9*init_altitude+0.1*baro.data(t,1);  
            balt=0; broc=0; 
            gps_px(t) = 0; gps_py(t) = 0;
            gps_vx(t) = 0; gps_vy(t) = 0;
        end
        
        %% update step
        if(mod(t,dtbaro*1000) == 0)
           h = @(x,args)[x(3); x(6);];
           z = [-balt; -broc]; % measurements
           [x, P] = ukf_update(dt,x,P,h,z,Rbaro,X,Xs);  % ukf measurement update
           X=[]; Xs=[]; % to avoid reusing it for next sensors
        end
        if((mod(t,dtgps*1000) == 0)&&(gps_index(t) ~= last_gps_index))
            last_gps_index = gps_index(t);
            h = @(x,args)[x(1); x(2); x(4); x(5);];
            z = [gps_px(t); gps_py(t); gps_vx(t); gps_vy(t);]; % measurements
            [x, P] = ukf_update(dt,x,P,h,z,Rgps,X,Xs);  % ukf pmeasurement update
        end
        %% save datasets
        if(df_id >= 0)
            fprintf(df_id, '%f %f %f %f %f %f %f %f %f %f %f %d %d\n', imu_roll(t), imu_pitch(t), imu_yaw(t), imu_ax(t), imu_ay(t), imu_az(t), baro_alt(t), gps_px(t),gps_py(t),gps_vx(t),gps_vy(t), gps_index(t), gps_status(t));
        end
        if(rf_id >= 0)
            fprintf(rf_id, '%f %f %f %f %f %f %f %f %f\n', x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9));
        end
    end
    %% save estimates
    xV(:,t) = x;
end
%% close dataset files
if(df_id >= 0)
   fclose(df_id);
end
if(rf_id >= 0)
   fclose(rf_id);
end
