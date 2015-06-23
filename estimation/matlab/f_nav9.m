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
% \file       f_nav9.m
% \brief      function to predict pos/vel given previous pos/vel and control input
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% function to predict pos/vel given previous pos/vel and control input
%  in:
%    x = [px, py, pz, vx, vy, vz, abx, aby, abz] = pos, vel, acc byas
%    u = [roll, pitch, yaw, ax, ay, az] = euler, acc = control input
%  out:
%    output = [px py pz vx vy vz abx aby abz] = pos, vel, acc byas
function output = f_nav9(x, u, dt)

px = x(1); py = x(2); pz = x(3); vx = x(4); vy = x(5); vz = x(6); abx = x(7); aby = x(8); abz = x(9); 
roll = u(1); pitch = u(2); yaw = u(3); ax = u(4); ay = u(5); az = u(6);

% rotate body acceleration to NED reference frame
[Ax, Ay, Az] = bodyToNed(roll, pitch, yaw, ax-abx, ay-aby, az-abz);

% state prediction
vx = vx + Ax*dt; 
vy = vy + Ay*dt; 
vz = vz + Az*dt;
px = px + vx*dt; 
py = py + vy*dt; 
pz = pz + vz*dt;
           
output = [ px; py; pz; vx; vy; vz; abx; aby; abz;];

end
