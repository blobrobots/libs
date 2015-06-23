%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The MIT License (MIT)
%
% Copyright (c) 2015 Blob Robotics
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal 
% in the Software without restriction, including without limitation the rights 
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is 
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
% SOFTWARE.
% 
% \file       cf_nav6_z42.m
% \brief      example of use of the cf library to estimate position and velocity 
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% example of use of the cf library to estimate position and velocity
%  x = [px, py, pz, vx, vy, vz] = [pos, vel];
%  u = [ax, ay, az];
%  z = [px, py, vx, vy] [pz, vz] = [pos,vel] [z,roc];

%% save datasets
%data_file = 'cf_nav6_z42.in';
%result_file = 'cf_nav6_z42.out';

%% sample times
dt     = 0.01; % filter sample time - 100Hz
dtgps  = 0.1;  % acc sample time - 50Hz
dtbaro = 0.1;  % mag sample time - 20Hz

%% filter gains
kp_vel = [2.5 2.5 2.5];
ki_vel = [0.01 0.01 0.1];
kp_pos = [1 1 1];
ki_pos = [0.01 0.01 0.01];

%% barometer pre-filter parameters
balt_max_rate = 0.05; %0.1;
broc_max_rate = 0.05; %0.05;

balt_lpf = 0.9;
broc_lpf = 0.0;

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

time = size(gps_index,1); % total dynamic steps (1 per ms)

%% initial values
f = @(x,args)f_nav3vel(x,args.u,args.dt);   % prediction equation
h = @(x,args)he_nav2vel(x,args.z);          % measurement equation
x = zeros(6,1);     % initial state
N = size(x,1);      % number of states   
ev = [0 0 0 0 0 0]; % initial u error
uv = [0 0 0];       % initial input
ep = [0 0 0 0 0 0];
z = [0;0;0;];     % initial measurement

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
   fprintf(rf_id, 'dt=%f dtgps=%f dtbaro=%f kp_baro=%f kp_gps=%f ki=%f\n',dt,dtgps,dtbaro,kp_baro,kp_gps,ki);
   fprintf(rf_id, 'px py pz vx vy vz\n');
end

%% loop
for t=1:time
    if (mod(t,dt*1000) == 0)
        %% update pre-filtered sensor data        
        balt = balt + constrain((balt_lpf*balt + (1-balt_lpf)*(baro_alt(t)-init_altitude))-balt, -balt_max_rate, balt_max_rate);
        broc = broc + constrain((broc_lpf*broc + (1-broc_lpf)*(balt-prev_balt)/dt) - broc, -broc_max_rate, broc_max_rate);
        prev_balt = balt;
        
        %% prediction step
        % control input is accelerations in NED frame
        f = @(x,args)f_nav3vel(x,args.u,args.dt);
        uv = bodyToNed(imu_roll(t), imu_pitch(t), imu_yaw(t), imu_ax(t), imu_ay(t), imu_az(t));
        [x,ev]= cf_predict(dt,x,ev,f,uv,ki_vel); % prediction step vel
        % control input is velocity in NED frame
        f = @(x,args)f_nav3pos(x,args.u,args.dt);
        up = x(4:6)';
        [x,ep]= cf_predict(dt,x,ep,f,up,ki_pos); % prediction step pos
        
        %% update step
        if(gps_status(t) < 1) % gps status not OK
            init_altitude = 0.9*init_altitude+0.1*baro.data(t,1);  
            balt=0; broc=0; 
            gps_px(t) = 0; gps_py(t) = 0;
            gps_vx(t) = 0; gps_vy(t) = 0;
        end
        
        if(mod(t,dtbaro*1000) == 0)
           h = @(x,args)he_nav1roc(x,args.z);
           z = [-broc;]; % roc measurements
           [x, ev] = cf_update(dtbaro,x,ev,h,z,kp_vel);  % cf roc measurement update
           h = @(x,args)he_nav1baro(x,args.z);
           z = [-balt;]; % baro measurements
           [x, ep] = cf_update(dtbaro,x,ep,h,z,kp_pos);  % cf height measurement update
        end
        if((mod(t,dtgps*1000) == 0)&&(gps_index(t) ~= last_gps_index))
            last_gps_index = gps_index(t);
            h = @(x,args)he_nav2vel(x,args.z);
            z = [gps_vx(t); gps_vy(t);]; % vel measurements
            [x, ev] = cf_update(dtgps,x,ev,h,z,kp_vel);  % cf vel measurement update
            h = @(x,args)he_nav2gps(x,args.z);
            z = [gps_px(t); gps_py(t);]; % gps measurements
            [x, ep] = cf_update(dtgps,x,ep,h,z,kp_pos);  % cf pos measurement update
        end
        %% save datasets
        if(df_id >= 0)
            fprintf(df_id, '%f %f %f %f %f %f %f %f %f %f %f %d %d\n', imu_roll(t), imu_pitch(t), imu_yaw(t), imu_ax(t), imu_ay(t), imu_az(t), baro_alt(t), gps_px(t),gps_py(t),gps_vx(t),gps_vy(t), gps_index(t), gps_status(t));
        end
        if(rf_id >= 0)
            fprintf(rf_id, '%f %f %f %f %f %f\n', x(1), x(2), x(3), x(4), x(5), x(6));
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
