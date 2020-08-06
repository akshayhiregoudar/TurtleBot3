classdef ExampleHelperAMCLWanderer
    %EXAMPLEHELPERAMCLWANDERER Robot driver using Vector Field Histogram
    %method.
    % w = ExampleHelperAMCLWanderer(laserSub, tf, velPub, velMsg) creates a
    % helper that drives robot using Vector Field Histogram (VFH) method.
    % VFH needs a laser scan subscriber, a transformation from laser sensor
    % frame to robot base frame, velocity publisher and the
    % corresponding message type to control a robot.
    
    % Copyright 2015-2017 The MathWorks, Inc.
    
    properties (Access = private)
        % LaserSub - Sensor Interface for laser scan.
        LaserSub
        % TF - Transformation Tree data
        TF
        % VelocityPublisher - Velocity publisher
        VelocityPublisher
        % VelocityMessage - ROS velocity message type
        VelocityMessage
        % Vector Field Histogram object
        VFH
        % Desired moving direction
        TargetDirection = 0
    end
    
    methods
        function obj = ExampleHelperAMCLWanderer(laserSub, tf, velPub, velMsg)
            
            % Initialize sensor interface
            obj.LaserSub = laserSub;
            obj.TF = tf;
            
            % Initialize control interface
            obj.VelocityPublisher = velPub;
            obj.VelocityMessage = velMsg;
            
            % Initialize VectorFieldHistogram object. The parameters here
            % are tuned to prevent TurtleBot pass doors in the office
            % environment used in AdaptiveMonteCarloLocalizationExample.
            obj.VFH = robotics.VectorFieldHistogram;
            obj.VFH.UseLidarScan = true;
            obj.VFH.DistanceLimits = [0.05 4];
            obj.VFH.RobotRadius = 0.4;
            obj.VFH.MinTurningRadius = 0.5;
            obj.VFH.SafetyDistance = 0.15;
            obj.VFH.HistogramThresholds= [1 8];
        end
        
        function wander(obj)
            %WANDER Move robot around while avoiding obstacles.
            
            % Get laser scan data
            transScan = transformLaserToRobot(obj, obj.LaserSub, obj.TF);
            
            % Call VFH step method
            steerDir = step(obj.VFH, transScan, obj.TargetDirection);
            
            if ~isnan(steerDir)
                desiredV = 0.25;
                w = exampleHelperComputeAngularVelocity(steerDir, 0.1);
            else
                desiredV = 0.0;
                w = 0.25;
            end
            
            % Send velocity commands
            drive(obj, desiredV, w)
        end
        
        function stop(obj)
            %STOP Stop the robot
            drive(obj, 0, 0);
        end
    end
    
    methods (Access = private)
        function drive(obj, v, w)
            %DRIVE Send out linear and angular velocities to robot through
            %ros publisher
            obj.VelocityMessage.Linear.X = v;
            obj.VelocityMessage.Angular.Z = w;
            send(obj.VelocityPublisher, obj.VelocityMessage);
        end
    end
    
    methods (Access = private)
        function transScan = transformLaserToRobot(~, scansub,tf)
            %transformLaserToRobot Transform laser data to robot base frame
            %  This transform function is only suitable for 2D laser scan
            %  sensor mounted on surface parallel to ground plane.
            
            % Receive laser scan
            scanMsg = receive(scansub);
            
            scan = lidarScan(scanMsg);
           
            % Get the rotation
            quat = [tf.Transform.Rotation.W tf.Transform.Rotation.X ...
                tf.Transform.Rotation.Y tf.Transform.Rotation.Z];
            odomRotation = quat2eul(quat);

            % Transform scan and return
            transScan = transformScan(scan, [tf.Transform.Translation.X tf.Transform.Translation.Y odomRotation(1)]);
        end
    end
    
end

