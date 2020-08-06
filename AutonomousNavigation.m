rosinit("http://ubuntu:11311")

% Subscribe to laser scan and velocity
laserSub = rossubscriber('/scan');
[velPub,velMsg] = rospublisher('/cmd_vel');

% Create an odometry motion model object
odometryModel = odometryMotionModel;

% Apply Guassian noise for vehicle motion
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Create ROS transformation tree
tftree = rostf;
pause(1);

% Wait until the transformation between the specified frames is available
waitForTransform(tftree,'/base_footprint', '/odom');

% Get the transformation
sensorTransform = getTransform(tftree,'/base_footprint', '/odom');


% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup Vector-Field Histogram (VFH) parameters
vfh = controllerVFH;
vfh.UseLidarScan = true;

% Lower distance limit is used to ignore false positives and the upper limit 
% to ignore obstacles that are too far from the robot
vfh.DistanceLimits = [0.05 1];

% Set radius of the robot
vfh.RobotRadius = 0.25;

% Set minimum turning radius of the robot
vfh.MinTurningRadius = 0.2;

% Safety distance for better obstacle aviodance
vfh.SafetyDistance = 0.1;

% Steering direction is forward if set to 0
targetDir = 0;

i =1;

while i < 201   % For 200 scans (or set accordingly)

	% Get laser scan data
	laserScan = receive(laserSub);
	ranges = double(laserScan.Ranges);
	angles = double(laserScan.readScanAngles);
 
	% Create a lidarScan object from the ranges and angles
    scan = lidarScan(ranges,angles);
    
    % Store all the scan readings in a matrix
    scans(:,i) = lidarScan(laserScan);
        
	% Call VFH object and compute the steering direction
	steerDir = vfh(scan, targetDir);  
    
	% Calculate the velocities
	if ~isnan(steerDir)    % If steering direction is valid
		desiredV = 0.45;
		w = exampleHelperComputeAngularVelocity(steerDir, 1);
    else    % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.25;
	end

	% Send velocity commands
	velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	velPub.send(velMsg);
    
    % Iterate
    i = i+1;
    
    % Set scan frequency
    pause(0.125);
end

% Disconnect from ROS network
rosshutdown