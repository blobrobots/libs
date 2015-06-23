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
% x = [q0, q1, q2, q3, gbx, gby, gbx] = quaternion, gyro bias
% z = [ax, ay, az] = acc
% out:
% output = [ax ay az] = expected acc given state
function output = h_imu3qa(x)

% normalize
q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);

% expected acc measurement given state estimation and direction of gravity in NED frame
output = [ 2*(q0*q2 - q1*q3);               % ax
          -2*(q0*q1 + q2*q3);               % ay
          -q0*q0 + q1*q1 + q2*q2 - q3*q3;]; % az
end

% explanation:
%
% navigation to body frame rotation of vector V given rotation quaternion:
% qV = [0 Vx Vy Vz]; qv = q'*qV*q
% where conjugate quaternion:
% q' = (q0 - q1*i - q2*j - q3*k)
% quaternion multiplication (Hamilton Product) 
% http://en.wikipedia.org/wiki/Quaternion#Hamilton_product):
% q = (q0 + q1*i + q2*j + q3*k)
% p = (p0 + p1*i + p2*j + p3*k)
% r = (r0 + r1*i + r2*j + r3*k)
% q = p x r ->
% q0 = (p0*r0 - p1*r1 - p2*r2 - p3*r3) = (r0*p0 - r1*p1 - r2*p2 - r3*p3)
% q1 = (p0*r1 + p1*r0 + p2*r3 - p3*r2) = (r0*p1 + r1*p0 - r2*p3 + r3*p2) 
% q2 = (p0*r2 - p1*r3 + p2*r0 + p3*r1) = (r0*p2 + r1*p3 + r2*p0 - r3*p1)
% q3 = (p0*r3 + p1*r2 - p2*r1 + p3*r0) = (r0*p3 - r1*p2 + r2*p1 + r3*p0)
% 
% measurement calculation:
%  
% normalized gravity vector in NED reference frame: g = [0 0 -1] (NED)
% equivalent quaternion: g = (0 + 0*i + 0*j - 1*k)
% expected normalized accelerometer measurement (body frame rotation):
% qa = q'*g*q =  +-q3 --q2 +-q1 -q0 = (-q3 +q2 -q1 -q0)*q =
%    = -q3*q0 - q2*q1 + q1*q2 + q0*q3
%      -q3*q1 + q2*q0 - q1*q3 + q0*q2
%      -q3*q2 - q2*q3 - q1*q0 - q0*q1
%      -q3*q3 + q2*q2 + q1*q1 - q0*q0 =
% 0 + 2*(q0*q2 - q1*q3)*i - 2*(q0*q1 + q2*q3)*j + (-q0*q0 + q1*q1 + q2*q2 - q3*q3)*k;
%
% normalized gravity vector in ENU reference frame: g = [0 0 1] (ENU)
% equivalent quaternion: g = (0 + 0*i + 0*j + 1*k)
% expected normalized accelerometer measurement (body frame rotation):
% qa = q'*g*q = 
% 0 + 2*(q1*q3 - q0*q2)*i + 2*(q0*q1 + q2*q3)*j + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*k;
