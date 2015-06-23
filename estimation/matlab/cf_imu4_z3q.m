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
% \file       cf_imu4_z3q.m
% \brief      example of use of the cf library to estimate attitude
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% example of use of the cf library to estimate attitude
%  x = [q0, q1, q2, q3] = [quaternion];
%  u = [gx, gy, gz] = [gyro];
%  z = [ax, ay, az], [mx, my, mz] = [acc], [mag];

%% save datasets
%data_file = 'cf_imu4_z3q.in';
%result_file = 'cf_imu4_z3q.out';

%% sample times
dt    = 0.01; % filter sample time - 100Hz
dtacc = 0.02; % acc sample time - 50Hz
dtmag = 0.05; % mag sample time - 20Hz

%% gains
kp_acc = [0.01 0.01 0.01];
kp_mag = [0.01 0.01 0.01];
ki = [0.0001 0.0001 0.0001];

%% pre-filter parameters
gyro_max_rate = 1;
acc_max_rate = 1;
mag_max_rate = 1;

gyro_lpf = 0;
acc_lpf = 0;
mag_lpf = 0;

%% input data
imu_gx = imu(2).data(:,1);
imu_gy = imu(2).data(:,2);
imu_gz = imu(2).data(:,3);
imu_ax = imu(2).data(:,4);
imu_ay = imu(2).data(:,5);
imu_az = imu(2).data(:,6);
imu_mx = imu(2).data(:,7);
imu_my = imu(2).data(:,8);
imu_mz = imu(2).data(:,9);

time = size(imu(2).data,1); % total dynamic steps (1 per ms)

%% initial values
f = @(x,args)f_imu4q(x,args.u,args.dt); % prediction equation
h = @(x,args)he_imu3qa(x,args.z);               % measurement equation
x = [1; 0; 0; 0;];                      % initial state q = [1 0 0 0]
N = size(x,1);                          % number of states                           
e = [0 0 0 0 0 0];                      % initial error
u = [0 0 0];                            % initial input
z = [0;0;-1];                           % normalized measurement

xV = zeros(N,time); % state output
eV = zeros(3,time); % euler angles output

ax = 0; ay = 0; az = 0;
mx = 0; my = 0; mz = 0;

%% save datasets
df_id = -1; rf_id = -1;
if(exist('data_file') == 1)
   df_id = fopen(data_file,'w');
end
if(exist('result_file') == 1)
   rf_id = fopen(result_file,'w');
end
if(df_id >= 0)
   fprintf(df_id, 'gx gy gz ax ay az mx my mz (dt=%f)\n',dt);
end
if(rf_id >= 0)
   fprintf(rf_id, 'dt=%f dtacc=%f dtmag=%f racc=%f rmag=%f qbg=%f\n',dt,dtacc,dtmag,racc,rmag,qbg);
   fprintf(rf_id, 'q0 q1 q2 q3 gbx gby gbz roll pitch yaw roll_gt pitch_gt yaw_gt\n');
end

%% loop
for t=1:time
    if (mod(t,dt*1000) == 0)
        %% filter sensor data
        ax = ax + constrain((acc_lpf*ax+(1-acc_lpf)*imu_ax(t)) - ax, -acc_max_rate, acc_max_rate);
        ay = ay + constrain((acc_lpf*ay+(1-acc_lpf)*imu_ay(t)) - ay, -acc_max_rate, acc_max_rate);
        az = az + constrain((acc_lpf*az+(1-acc_lpf)*imu_az(t)) - az, -acc_max_rate, acc_max_rate);
        
        mx = mx + constrain((mag_lpf*mx+(1-mag_lpf)*imu_mx(t)) - mx, -mag_max_rate, mag_max_rate);
        my = my + constrain((mag_lpf*my+(1-mag_lpf)*imu_my(t)) - my, -mag_max_rate, mag_max_rate);
        mz = mz + constrain((mag_lpf*mz+(1-mag_lpf)*imu_mz(t)) - mz, -mag_max_rate, mag_max_rate);
        
        %% update sensor data
        axn = ax; %imu_ax(t);
        ayn = ay; %imu_ay(t);
        azn = az; %imu_az(t);
        
        mxn = mx; %imu_mx(t);
        myn = my; %imu_my(t);
        mzn = mz; %imu_mz(t);

        %% normalise measurements
        anorm = sqrt(axn*axn + ayn*ayn + azn*azn);
        if (anorm > 0)
            axn = axn/anorm;
            ayn = ayn/anorm;
            azn = azn/anorm;
        end
        mnorm = sqrt(mxn*mxn + myn*myn + mzn*mzn);
        if (mnorm > 0)
            mxn = mxn/mnorm;
            myn = myn/mnorm;
            mzn = mzn/mnorm;
        end

        %% prediction step
        % control input is filtered gyro
        u(1) = u(1) + constrain((gyro_lpf*u(1)+(1-gyro_lpf)*imu_gx(t)) - u(1), -gyro_max_rate, gyro_max_rate);
        u(2) = u(2) + constrain((gyro_lpf*u(2)+(1-gyro_lpf)*imu_gy(t)) - u(2), -gyro_max_rate, gyro_max_rate);
        u(3) = u(3) + constrain((gyro_lpf*u(3)+(1-gyro_lpf)*imu_gz(t)) - u(3), -gyro_max_rate, gyro_max_rate);
        [x,e] = cf_predict(dt,x,e,f,u,ki);

        %% update step
        if(mod(t,dtacc*1000) == 0)
           h=@(x,args)he_imu3qa(x,args.z);
           z = [axn; ayn; azn]; % measurements
           [x, e] = cf_update(dtacc,x,e,h,z,kp_acc);
        end
        if(mod(t,dtmag*1000) == 0)
           h=@(x,args)he_imu3qm(x,args.z);
           z = [mxn; myn; mzn]; % measurements
           [x, e] = cf_update(dtmag,x,e,h,z,kp_mag);  % ukf predict + measurement update
        end
        
        
        %% re-normalize just in case
        qnorm = sqrt(x(1)*x(1) + x(2)*x(2) + x(3)*x(3) + x(4)*x(4));
        x(1) = x(1)/qnorm;
        x(2) = x(2)/qnorm;
        x(3) = x(3)/qnorm;
        x(4) = x(4)/qnorm;
    
        %% save datasets
        if(df_id >= 0)
            fprintf(df_id, '%f %f %f %f %f %f %f %f %f\n', imu_gx(t), imu_gy(t), imu_gz(t), imu_ax(t), imu_ay(t), imu_az(t), imu_mx(t), imu_my(t), imu_mz(t));
        end
        if(rf_id >= 0)
            fprintf(rf_id, '%f %f %f %f %f %f %f %f %f %f %f %f %f\n', x(1), x(2), x(3), x(4), x(5), x(6), x(7), atan2(2*(x(1)*x(2) + x(3)*x(4)), 1 - 2*(x(2)*x(2) + x(3)*x(3))), asin(2*(x(1)*x(3) - x(2)*x(4))), atan2(2*(x(1)*x(4) + x(2)*x(3)), 1 - 2*(x(3)*x(3) + x(4)*x(4))), imu(2).data(t,10), imu(2).data(t,11), imu(2).data(t,12));
        end
    end
    
    %% save estimates
    xV(:,t) = x;                            
    eV(1,t) = atan2(2*(x(1)*x(2) + x(3)*x(4)), 1 - 2*(x(2)*x(2) + x(3)*x(3)));
    eV(2,t) = asin(2*(x(1)*x(3) - x(2)*x(4)));
    eV(3,t) = atan2(2*(x(1)*x(4) + x(2)*x(3)), 1 - 2*(x(3)*x(3) + x(4)*x(4)));
end
%% close dataset files
if(df_id >= 0)
   fclose(df_id);
end
if(rf_id >= 0)
   fclose(rf_id);
end