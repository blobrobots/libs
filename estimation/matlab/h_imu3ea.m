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
% \file       h_imu3qa.m
% \brief      expected accelerometer measurement given state
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% expected accelerometer measurement given state
% in:
% x = [roll, pitch, yaw, gbx, gby, gbx] = euler, gyro bias
% z = [ax, ay, az] = acc
% out:
% output = [ax ay az] = expected acc given state
function output = h_imu3ea(x)

roll = x(1); pitch = x(2); yaw = x(3);

g = -1; % ned2frd

% expected acc measurement given state estimation and direction of gravity in NED frame
output = [ -g*sin(pitch);              % ax
            g*sin(roll)*cos(pitch);    % ay
            g*cos(pitch)*cos(roll); ]; % az
end

% explanation:
% 
% navigation to body frame rotation:
% bfx = nfx*cos(pitch)*cos(yaw)                                  + nfy*cos(pitch)*sin(yaw)                                  - nfz*sin(pitch);
% bfy = nfx*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)) + nfy*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw)) + nfz*sin(roll)*cos(pitch);
% bfz = nfx*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)) + nfy*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)) + nfz*cos(pitch)*cos(roll);
%
% normalized gravity vector in NED reference frame: g = [0; 0; -1]
% expected normalized accelerometer measurement (body frame rotation):
% a = [ sin(pitch); -sin(roll)*cos(pitch); -cos(pitch)*cos(roll);]
%