%% ukf_imu7_z3q.m
%  author: adrian jimenez gonzalez
%  email:  blobrobotics@gmail.com
%  date:   15-jan-2015
%  brief:  Example of use of the ukf library to estimate attitude

%% save datasets
data_file = 'ukf_imu7_z3q.in';
result_file = 'ukf_imu7_z3q.out';

%% tiempos de muestreo
dt = 0.01;    % freq de filtro 100Hz
dtacc = 0.02; % freq de acc 50Hz
dtmag = 0.05; % freq de mag 20Hz

%% caracterizacion sensores
racc = 0.1; %std of acc measurement
rmag = 0.25; %std of mag measurement

qq = 0; qbg = 0.0001;

gyro_max_rate = 1; %0.05;
acc_max_rate = 1;
mag_max_rate = 1;

gyro_lpf = 0;
acc_lpf = 0;
mag_lpf = 0;

q=[qq,qq,qq,qq,qbg,qbg,qbg];    %std of process: qpos qvel qacc
r=[racc,racc,racc,rmag,rmag,rmag];        %std of measurement: qpos qvel qacc

Q = [  (qq*dt)^2    0     0     0      0      0     0     % covariance of process
         0    (qq*dt)^2   0     0      0      0     0     
         0      0   (qq*dt)^2   0      0      0     0     
         0      0     0   (qq*dt)^2    0      0     0     
         0      0     0     0   (qbg*dt)^2    0     0    
         0      0     0     0      0   (qbg*dt)^2   0    
         0      0     0     0      0      0   (qbg*dt)^2 ];
     
Ra = [ (racc*dtacc)^2    0    0      % covariance of measurement
         0   (racc*dtacc)^2   0
         0      0  (racc*dtacc)^2 ];
     
Rm = [ (rmag*dtmag)^2   0     0      % covariance of measurement
         0   (rmag*dtmag)^2   0
         0      0  (rmag*dtmag)^2 ];

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

%% initial values
% IMU:
% x = [q0, q1, q2, q3, gxb, gyb, gzb] = [quaternion, gyro_bias];
% u = [gx, gy, gz] = [gyro];
% z = [ax, ay, az], [mx, my, mz] = [acc], [mag];

f = @(x,args)f_imu7q(x,args.u,args.dt); % prediction equation
h = @(x,args)h_imu3qa(x);               % measurement equation
x = [1; 0; 0; 0; 0; 0; 0;];             % initial state q = [1 0 0 0]
N = size(x,1);                          % number of states                           
P = eye(N);                             % initial state covariance
u = [0 0 0];                            % initial input
z = [0;0;-1];                           % normalized measurement

time=size(imu(2).data,1); % total dynamic steps (1 per ms)

xV = zeros(N,time); % state output
eV = zeros(3,time); % euler angles output

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

        u(1) = imu_gx(t);
        u(2) = imu_gy(t);
        u(3) = imu_gz(t);

        %% prediction step
        [x,P,X,Xs]= ukf_predict(dt,x,P,f,u,Q); % prediction step

        %% update step
        if(mod(t,dtacc*1000) == 0)
           h=@(x,args)h_imu3qa(x);
           z = [axn; ayn; azn]; % measurements
           [x, P] = ukf_update(dt,x,P,h,z,Ra,X,Xs);  % ukf predict + measurement update
           
           X=[]; Xs=[]; % to avoid reusing it for magnetometers
        end
        if(mod(t,dtmag*1000) == 0)
           h=@(x,args)h_imu3qm(x);
           z = [mxn; myn; mzn]; % measurements
           [x, P] = ukf_update(dt,x,P,h,z,Rm,X,Xs);  % ukf predict + measurement update
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