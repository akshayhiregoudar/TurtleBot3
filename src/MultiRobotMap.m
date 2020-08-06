% To build the map using the lidar scans

% Set lidar range (in m)
maxLidarRange = 12;

% Set map resolution (Greater the resolution, longer the build time)
mapResolution = 20;     % cells per meter

% Create a lidarSLAM object
slamAlg_0 = lidarSLAM(mapResolution, maxLidarRange);
slamAlg_1 = lidarSLAM(mapResolution, maxLidarRange);
slamAlg_2 = lidarSLAM(mapResolution, maxLidarRange);

slamAlg_0.LoopClosureThreshold = 210;     % threshold for accepting loop closures
slamAlg_0.LoopClosureSearchRadius = 12;   % radius for loop closure detection

slamAlg_1.LoopClosureThreshold = 210;
slamAlg_1.LoopClosureSearchRadius = 12;

slamAlg_2.LoopClosureThreshold = 210;
slamAlg_2.LoopClosureSearchRadius = 12;

% Add all the scans to the lidarSLAM object
for i=1:length(scans_0)
    [isScanAccepted_0, loopClosureInfo_0, optimizationInfo_0] = addScan(slamAlg_0, scans_0(i));
    if isScanAccepted_0
        fprintf('Added scan_0 %d \n', i);
    end
end

for i=1:length(scans_1)
    [isScanAccepted_1, loopClosureInfo_1, optimizationInfo_1] = addScan(slamAlg_1, scans_1(i));
    if isScanAccepted_1
        fprintf('Added scan_1 %d \n', i);
    end
end

for i=1:length(scans_2)
    [isScanAccepted_2, loopClosureInfo_2, optimizationInfo_2] = addScan(slamAlg_2, scans_2(i));
    if isScanAccepted_2
        fprintf('Added scan_2 %d \n', i);
    end
end


% figure;
% show(slamAlg_0);
% hold on;
% show(slamAlg_1);
% show(slamAlg_2);
% hold off;
% title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

figure(1);
ax1 = subplot(1, 3, 1);
show(slamAlg_0, "Parent", ax1);
title('Room 1');

ax2 = subplot(1, 3, 2);
show(slamAlg_2, "Parent", ax2);
title('Room 2');

ax3 = subplot(1, 3, 3);
show(slamAlg_1, "Parent", ax3);
title('Room 3');


% Build an occupancy map
mapscan_0 = scans_0; 
[mapscan_0, optimizedPoses_0]  = scansAndPoses(slamAlg_0);
map_0 = buildMap(mapscan_0, optimizedPoses_0, mapResolution, maxLidarRange);

mapscan_1 = scans_1; 
[mapscan_1, optimizedPoses_1]  = scansAndPoses(slamAlg_1);
map_1 = buildMap(mapscan_1, optimizedPoses_1, mapResolution, maxLidarRange);

mapscan_2 = scans_2; 
[mapscan_2, optimizedPoses_2]  = scansAndPoses(slamAlg_2);
map_2 = buildMap(mapscan_2, optimizedPoses_2, mapResolution, maxLidarRange);


% Display map
% figure;
% show(map_0);
% hold on;
% show(map_1);
% show(map_2);
% hold off;
% title('Occupancy Grid Map Built Using Lidar SLAM');

figure(2);
ax1 = subplot(1, 3, 1);
show(map_0, "Parent", ax1);
title('Room 1');
ax2 = subplot(1, 3, 2);
show(map_2, "Parent", ax2);
title('Room 2');
ax3 = subplot(1, 3, 3);
show(map_1, "Parent", ax3);
title('Room 3');