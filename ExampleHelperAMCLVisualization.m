classdef ExampleHelperAMCLVisualization < handle
    %EXAMPLEHELPERAMCLVISUALIZATION Visualize the localization process.
    % v = ExampleHelperAMCLVisualization(map) will create a visualization
    % help for AdaptiveMonteCarloLocalizationExample that plots robot 
    % position, AMCL particles and laser scan inside the parameter map.
    
    % Copyright 2015-2017 The MathWorks, Inc.
    
    properties (Access = private)
        %Figure - Graph handle for the visualization figure.
        Figure
        %Axes - Graph handle for the visualization axes.
        Axes
        %ParticlePlot - Graph handle for particle plot.
        ParticlePlot
        %PosePlot - Graph handle for robot position plot.
        PositionPlot
        %OrientationPlot - Graph handle for robot orientation plot.
        OrientationPlot
        %LaserPlot - Graph handle for laser scan plot.
        LaserPlot
    end
    
    methods
        function obj = ExampleHelperAMCLVisualization(og)
            
            % Initialize graph handles
            obj.ParticlePlot = [];
            obj.PositionPlot = [];
            obj.OrientationPlot = [];
            obj.LaserPlot = [];
            
            % Show BOG
            figure()
            obj.Figure = gcf;
            obj.Axes = gca;
            hold off
            %og.show(tb3map, 'Parent', obj.Axes);
            hold on
            
        end
        
        function plotStep(obj, amcl, estimatedPose, scan, numUpdates)
            %plotStep Update BOG plot with latest robot and particle data.
            % Plot the robot's estimated pose, particles and laser
            % scans on the BOG.
            
            % Get particles from AMCL.
            [particles, ~] = amcl.getParticles;
            % Compute the end point for robot orientation vector.
            orient = [estimatedPose(1) + cos(estimatedPose(3)), estimatedPose(2) + sin(estimatedPose(3))];
            % Transform laser scans from camera frame to global frame.
            transScan = transformScan(scan, amcl.SensorModel.SensorPose + estimatedPose);
            transScanCart = transScan.Cartesian;

            if ishandle(obj.Axes)
                if isempty(obj.ParticlePlot)
                    % Create plots inside obj.Figure and store plot handles
                    % Plot particles.
                    obj.ParticlePlot = plot(obj.Axes, particles(:, 1), particles(:,2), '.');
                    % Plot estimated robot position.
                    obj.PositionPlot = plot(obj.Axes, estimatedPose(1), estimatedPose(2), 'go');
                    % Plot estimated robot orientation.
                    obj.OrientationPlot = plot(obj.Axes, [estimatedPose(1),orient(1)], [estimatedPose(2),orient(2)], 'g');
                    % Plot laser scans
                    obj.LaserPlot = plot(obj.Axes, transScanCart(:,1), transScanCart(:,2), 'r.');
                else
                    % Update each plot: particle, position, orientation, laser
                    set(obj.ParticlePlot, 'XData', particles(:, 1), 'YData', particles(:, 2));
                    set(obj.PositionPlot, 'XData', estimatedPose(1), 'YData', estimatedPose(2));
                    set(obj.OrientationPlot, 'XData', [estimatedPose(1),orient(1)], 'YData', [estimatedPose(2),orient(2)]);
                    set(obj.LaserPlot, 'XData', transScanCart(:,1), 'YData', transScanCart(:,2));
                end
                % Show the number of AMCL updates so far.
                title(obj.Axes, ['AMCL update = ', num2str(numUpdates)]);
            end
        end
    end
end

