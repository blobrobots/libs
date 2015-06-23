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
% \file       ukf_imu6_z3e.m
% \brief      example of use of the ukf library to estimate attitude
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% example of use of the ukf library to estimate attitude
% x = [q0, q1, q2, q3, gxb, gyb, gzb] = [quaternion, gyro_bias];
% u = [gx, gy, gz] = [gyro];
% z = [ax, ay, az], [mx, my, mz] = [acc], [mag];

%% save datasets
%data_file = 'ukf_imu6_z3e.in';
%result_file = 'ukf_imu6_z3e.out';

%% sample times
dt    = 0.01; % filter sample time - 100Hz
dtacc = 0.02; % acc sample time - 50Hz
dtmag = 0.05; % mag sample time - 20Hz

%% caracterizacion sensores
racc = 0.1; %std of acc measurement
rmag = 0.5; %std of mag measurement

qe = 0; qbg = 0.0001; % std dev of process

Q = [  (qe*dt)^2    0     0     0      0      0
         0    (qe*dt)^2   0     0      0      0        
         0      0   (qe*dt)^2   0      0      0       
         0      0     0   (qbg*dt)^2    0     0       
         0      0     0     0   (qbg*dt)^2    0       
         0      0     0     0      0   (qbg*dt)^2 ]; % covariance of process
     
Ra = [ (racc*dtacc)^2   0     0     
         0   (racc*dtacc)^2   0
         0      0  (racc*dtacc)^2 ]; % covariance of acc measurement
     
Rm = [ (rmag*dtmag)^2   0     0     
         0   (rmag*dtmag)^2   0
         0      0  (rmag*dtmag)^2 ]; % covariance of mag measurement

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

time=size(imu(2).data,1); % total dynamic steps (1 per ms)

%% initial values
f = @(x,args)f_imu6e(x,args.u,args.dt); % prediction equation
h = @(x,args)h_imu3ea(x);               % measurement equation
x = zeros(6,1);                         % initial state
N = size(x,1);                          % number of states                           
P = eye(N);                             % initial state covariance
u = [0 0 0];                            % initial input
z = [0;0;-1];                           % normalized measurement

xV = zeros(N,time); % state output

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
   fprintf(rf_id, 'q0 q1 q2 q3 gbx gby gbz roll pitch yaw\n');
end

%% loop
for t=1:time
    if (mod(t,dt*1000) == 0)
        %% update sensor data
        axn = imu_ax(t);
        ayn = imu_ay(t);
        azn = imu_az(t);
        
        mxn = imu_mx(t);
        myn = imu_my(t);
        mzn = imu_mz(t);

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
        u(1) = imu_gx(t);
        u(2) = imu_gy(t);
        u(3) = imu_gz(t);
        [x,P,X,Xs]= ukf_predict(dt,x,P,f,u,Q);
        
        %% update step
        if(mod(t,dtacc*1000) == 0)
           h=@(x,args)h_imu3ea(x);
           z = [axn; ayn; azn]; % accelerometer measurements
           [x, P] = ukf_update(dtacc,x,P,h,z,Ra,X,Xs);
           X=[]; Xs=[]; % to avoid reusing it for magnetometers
        end
        if(mod(t,dtmag*1000) == 0)
           h=@(x,args)h_imu3em(x);
           z = [mxn; myn; mzn]; % magnetometer measurements
           [x, P] = ukf_update(dtmag,x,P,h,z,Rm,X,Xs);
        end
        
        %% save datasets
        if(df_id >= 0)
            fprintf(df_id, '%f %f %f %f %f %f %f %f %f\n', imu_gx(t), imu_gy(t), imu_gz(t), imu_ax(t), imu_ay(t), imu_az(t), imu_mx(t), imu_my(t), imu_mz(t));
        end
        if(rf_id >= 0)
            fprintf(rf_id, '%f %f %f %f %f %f\n', x(1), x(2), x(3), x(4), x(5), x(6));
        end
    end
    % plot purposes
    xV(:,t) = x;                            
end
%% close dataset files
if(df_id >= 0)
   fclose(df_id);
end
if(rf_id >= 0)
   fclose(rf_id);
end