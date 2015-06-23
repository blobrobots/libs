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
% \file       he_imu3ea.m
% \brief      error update function for attitude estimation using accelerometer
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% error update function for attitude estimation using accelerometer
% in:
% x = [roll, pitch, yaw] = euler
% z = [ax, ay, az] = accelerometer
% out:
% error = [gx gy gz] = gyro
function error = he_imu3ea(x,z)

z1 = h_imu3ea(x); % expected measurement

% weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
norm = sqrt(z(1)*z(1) + z(2)*z(2) + z(3)*z(3));
aw = constrain(1 - 2*abs(1 - norm),0,1);

% error is sum of cross product between reference direction of field and direction measured by sensor
error = [ aw*(z(2)*z1(3) - z(3)*z1(2));
          aw*(z(3)*z1(1) - z(1)*z1(3));
          aw*(z(1)*z1(2) - z(2)*z1(1)); ];
end