rosinit("http://ubuntu:11311")

% Subscribe to laser scan and velocity
laserSub_0 = rossubscriber('/tb3_0/scan');
[velPub_0,velMsg_0] = rospublisher('/tb3_0/cmd_vel');

laserSub_1 = rossubscriber('/tb3_1/scan');
[velPub_1,velMsg_1] = rospublisher('/tb3_1/cmd_vel');

laserSub_2 = rossubscriber('/tb3_2/scan');
[velPub_2,velMsg_2] = rospublisher('/tb3_2/cmd_vel');


% Create an odometry motion model object
odometryModel = odometryMotionModel;

% Apply Guassian noise for vehicle motion
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Create ROS transformation tree
tftree = rostf;
pause(1);

% Wait until the transformation between the specified frames is available
waitForTransform(tftree,'/tb3_0/base_footprint', '/tb3_0/odom');
waitForTransform(tftree,'/tb3_1/base_footprint', '/tb3_1/odom');
waitForTransform(tftree,'/tb3_2/base_footprint', '/tb3_2/odom');


% Get the transformation
sensorTransform_0 = getTransform(tftree,'/tb3_0/base_footprint', '/tb3_0/odom');
sensorTransform_1 = getTransform(tftree,'/tb3_1/base_footprint', '/tb3_1/odom');
sensorTransform_2 = getTransform(tftree,'/tb3_2/base_footprint', '/tb3_2/odom');


% Get the euler rotation angles.
laserQuat_0 = [sensorTransform_0.Transform.Rotation.W sensorTransform_0.Transform.Rotation.X sensorTransform_0.Transform.Rotation.Y sensorTransform_0.Transform.Rotation.Z];
laserRotation_0 = quat2eul(laserQuat_0, 'ZYX');

laserQuat_1 = [sensorTransform_1.Transform.Rotation.W sensorTransform_1.Transform.Rotation.X sensorTransform_1.Transform.Rotation.Y sensorTransform_1.Transform.Rotation.Z];
laserRotation_1 = quat2eul(laserQuat_1, 'ZYX');

laserQuat_2 = [sensorTransform_2.Transform.Rotation.W sensorTransform_2.Transform.Rotation.X sensorTransform_2.Transform.Rotation.Y sensorTransform_2.Transform.Rotation.Z];
laserRotation_2 = quat2eul(laserQuat_2, 'ZYX');

% Setup Vector-Field Histogram (VFH) parameters
vfh = controllerVFH;
vfh.UseLidarScan = true;

% Lower distance limit is used to ignore false positives and the upper limit 
% to ignore obstacles that are too far from the robot
vfh.DistanceLimits = [0.05 1];

% Set radius of the robot
vfh.RobotRadius = 0.25;

% Set minimum turning radius of the robot
vfh.MinTurningRadius = 0.5;

% Safety distance for better obstacle aviodance
vfh.SafetyDistance = 0.25;

% Steering direction is forward if set to 0
targetDir = 0;

i =1;

while i < 51   % For 200 scans (or set accordingly)

	% Get laser scan data
	laserScan_0 = receive(laserSub_0);
	ranges_0 = double(laserScan_0.Ranges);
	angles_0 = double(laserScan_0.readScanAngles);
    
    laserScan_1 = receive(laserSub_1);
	ranges_1 = double(laserScan_1.Ranges);
	angles_1 = double(laserScan_1.readScanAngles);
    
    laserScan_2 = receive(laserSub_2);
	ranges_2 = double(laserScan_2.Ranges);
	angles_2 = double(laserScan_2.readScanAngles);
 
    
	% Create a lidarScan object from the ranges and angles
    scan_0 = lidarScan(ranges_0,angles_0);
    scan_1 = lidarScan(ranges_1,angles_1);
    scan_2 = lidarScan(ranges_2,angles_2);
    
    
    % Store all the scan readings in a matrix
    scans_0(:,i) = lidarScan(laserScan_0);
    scans_1(:,i) = lidarScan(laserScan_1);
    scans_2(:,i) = lidarScan(laserScan_2);
    
        
	% Call VFH object and compute the steering direction
	steerDir_0 = vfh(scan_0, targetDir);
    steerDir_1 = vfh(scan_1, targetDir);
    steerDir_2 = vfh(scan_2, targetDir);
    
    
	% Calculate the velocities
	if ~isnan(steerDir_0)    % If steering direction is valid
		desiredV_0 = 0.5;
		w_0 = exampleHelperComputeAngularVelocity(steerDir_0, 1);
    else    % Stop and search for valid direction
		desiredV_0 = 0.0;
		w_0 = 0.35;
    end
    
    if ~isnan(steerDir_1)    % If steering direction is valid
		desiredV_1 = 0.5;
		w_1 = exampleHelperComputeAngularVelocity(steerDir_1, 1);
    else    % Stop and search for valid direction
		desiredV_1 = 0.0;
		w_1 = 0.35;
    end
    
    if ~isnan(steerDir_2)    % If steering direction is valid
		desiredV_2 = 0.5;
		w_2 = exampleHelperComputeAngularVelocity(steerDir_2, 1);
    else    % Stop and search for valid direction
		desiredV_2 = 0.0;
		w_2 = 0.35;
    end
    

	% Send velocity commands
	velMsg_0.Linear.X = desiredV_0;
	velMsg_0.Angular.Z = w_0;
	velPub_0.send(velMsg_0);
    
    velMsg_1.Linear.X = desiredV_1;
	velMsg_1.Angular.Z = w_1;
	velPub_1.send(velMsg_1);
    
    velMsg_2.Linear.X = desiredV_2;
	velMsg_2.Angular.Z = w_2;
	velPub_2.send(velMsg_2);
    % Iterate
    i = i+1;
    
    % Set scan frequency
    pause(0.1);
end

% Disconnect from ROS network
rosshutdown
