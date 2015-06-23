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
% \file       ukf_nav9_z21.m
% \brief      example of use of the ukf library to estimate position and velocity 
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% example of use of the ukf library to estimate position and velocity
% x = [px, py, pz, vx, vy, vz, axb, ayb, azb] = [pos, vel, acc_bias];
% u = [roll, pitch, yaw, ax, ay, az];
% z = [px, py] [pz] = [pos] [alt];

%% save datasets
%data_file = 'ukf_nav9_z21.in';
%result_file = 'ukf_nav9_z21.out';

%% sample times
dt     = 0.01; % filter sample time - 100Hz
dtgps  = 0.1;  % gps sample time - 10Hz
dtbaro = 0.1;  % baro sample time - 10Hz

%% caracterizacion sensores
rgps  = 0.001; %std dev of gps measurement
rbaro = 0.005; %std dev of baro measurement

% std dev of process
qpx = 0; qpy = 0; qpz = 0;
qvx = 0; qvy = 0; qvz = 0;
qba = 0.001;

Q = [  (qpx*dt)^2    0     0     0      0      0     0     0     0   
         0    (qpy*dt)^2   0     0      0      0     0     0     0
         0      0   (qpz*dt)^2   0      0      0     0     0     0
         0      0     0   (qvx*dt)^2    0      0     0     0     0
         0      0     0     0    (qvy*dt)^2    0     0     0     0
         0      0     0     0      0    (qvz*dt)^2   0     0     0
         0      0     0     0      0      0   (qba*dt)^2   0     0
         0      0     0     0      0      0     0   (qba*dt)^2   0
         0      0     0     0      0      0     0     0   (qba*dt)^2 ]; % covariance of process
     
Rgps = [ (rgps*dtgps)^2    0      
            0   (rgps*dtgps)^2   ]; % covariance of gps measurement
        
Rbaro = [ (rbaro*dtbaro)^2    ];    % covariance of baro measurement

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
f = @(x,args)f_nav9(x,args.u,args.dt);   % prediction equation
h = @(x,args)[x(1); x(2);];  % measurement equation
x = zeros(9,1);          % initial state
N = size(x,1);           % number of states   
P = eye(N);              % initial state covraiance
u = [0 0 0 0 0 -9.8];    % initial input
z = [0;0;];          % initial measurement

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

        %% prediction step
        u(1) = imu_roll(t);  % roll
        u(2) = imu_pitch(t); % pitch
        u(3) = imu_yaw(t);   % yaw
        u(4) = imu_ax(t);    % roll
        u(5) = imu_ay(t);    % pitch
        u(6) = imu_az(t);    % yaw
        [x,P,X,Xs]= ukf_predict(dt,x,P,f,u,Q); % prediction step
        
        %% update step
        if(gps_status(t) < 1) % gps status not OK
            init_altitude = 0.9*init_altitude+0.1*baro.data(t,1);  
            balt=0; broc=0; 
            gps_px(t) = 0; gps_py(t) = 0;
            gps_vx(t) = 0; gps_vy(t) = 0;
        end
        if(mod(t,dtbaro*1000) == 0)
           h = @(x,args)[x(3);];
           z = [-balt;]; % measurements
           [x, P] = ukf_update(dtbaro,x,P,h,z,Rbaro,X,Xs);  % ukf measurement update
           X=[]; Xs=[]; % to avoid reusing it for next sensors
        end
        if((mod(t,dtgps*1000) == 0)&&(gps_index(t) ~= last_gps_index))
            last_gps_index = gps_index(t); % update last index processed
            h = @(x,args)[x(1); x(2);];
            z = [gps_px(t); gps_py(t);]; % measurements
            [x, P] = ukf_update(dtgps,x,P,h,z,Rgps,X,Xs);  % ukf measurement update
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
