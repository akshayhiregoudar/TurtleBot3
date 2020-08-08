clear;
clc;

rosinit("http://ubuntu:11311")

robot = rospublisher('/cmd_vel');
velmsg = rosmessage(robot);

laser = rossubscriber('/scan');


spinVelocity = 0.3;        % Angular velocity (rad/s)
forwardVelocity = 0.8;     % Linear velocity (m/s)
backwardVelocity = -0.2;   % Linear velocity (reverse) (m/s)
distanceThreshold = 0.4;  % Distance threshold (m) for turning

r = rateControl(5);

tic;
 while toc < 60
     % Collect information from laser scan
     scan = receive(laser);
     plot(scan);
     data = readCartesian(scan);
     x = data(:,1);
     y = data(:,2);
     % Compute distance of the closest obstacle
     dist = sqrt(x.^2 + y.^2);
     minDist = min(dist);     
     % Command robot action
     if minDist < distanceThreshold
         % If close to obstacle, back up slightly and spin
         velmsg.Angular.Z = spinVelocity;
         velmsg.Linear.X = backwardVelocity;
     else
         % Continue on forward path
         velmsg.Linear.X = forwardVelocity;
         velmsg.Angular.Z = 0;
         
     end   
     send(robot,velmsg);
     
     waitfor(r);
 end
 
 rosshutdown