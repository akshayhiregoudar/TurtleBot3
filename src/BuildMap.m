% To build the map using the lidar scans

% Set lidar range (in m)
maxLidarRange = 12;

% Set map resolution (Greater the resolution, longer the build time)
mapResolution = 30;     % cells per meter

% Create a lidarSLAM object
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;     % threshold for accepting loop closures
slamAlg.LoopClosureSearchRadius = 12;   % radius for loop closure detection

% Add all the scans to the lidarSLAM object
for i=1:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans(i));
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

%firstTimeLCDetected = false;

figure;
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

% Build an occupancy map
mapscan = scans; 
[mapscan, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(mapscan, optimizedPoses, mapResolution, maxLidarRange);

% Display
figure; 
show(map);
title('Occupancy Grid Map Built Using Lidar SLAM');