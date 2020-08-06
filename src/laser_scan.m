rosinit("http://ubuntu:11311")

%show(tb3map)

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

%rangeFinderModel = likelihoodFieldSensorModel;
%rangeFinderModel.SensorLimits = [0.1 8];
%rangeFinderModel.Map = tb3map;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
pause(1);
waitForTransform(tftree,'/base_footprint', '/odom');
sensorTransform = getTransform(tftree,'/base_footprint', '/odom');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
%rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
%amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

%visualizationHelper = ExampleHelperAMCLVisualization(tb3map);

wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

numUpdates = 25;
i = 1;
while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
%     % Create lidarScan object to pass to the AMCL object.
     scan = lidarScan(scanMsg);
%     
%     % For sensors that are mounted upside down, you need to reverse the
%     % order of scan angle readings using 'flip' function.
%     
%     % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
%     
%     % Update estimated robot's pose and covariance using new odometry and
%     % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
%     
%     % Drive robot to next pose.
    wander(wanderHelper);
%     
%     % Plot the robot's estimated pose, particles and laser scans on the map.
%     if isUpdated
%         i = i + 1;
%         plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
%     end
    scans(:,i) = lidarScan(scanMsg);
end