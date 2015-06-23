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
% \file       h_imu3qm.m
% \brief      expected magnetometer measurement given state
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% expected magnetometer measurement given state
% in:
% x = [q0, q1, q2, q3, gbx, gby, gbx] = quaternion, gyro bias
% z = [mx, my, mz] = mag
% out:
% output = [mx, my, mz] = expected mag given state
function output = h_imu3qm(x)

% normalize
q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);

% expected mag measurement given state estimation and direction of magnetic field in NED frame
output = [ q0*q0 + q1*q1 - q2*q2 - q3*q3; % mx
           2*(q1*q2 - q0*q3);             % my
           2*(q0*q2 + q1*q3);          ]; % mz
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
% normalized magnetic field vector in navigation reference frame: h = [1 0 0]
% equivalent quaternion: h = (0 1*i 0*j 0*k)
% expected normalized magnetometer measurement (body frame rotation):
% qm = q'*h*q =
% 0 + (q0*q0 + q1*q1 - q2*q2 - q3*q3)*i + 2*(q1*q2 - q0*q3)*j + 2*(q0*q2 + q1*q3)*k;
