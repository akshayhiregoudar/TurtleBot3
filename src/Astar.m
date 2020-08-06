% Load the map of the environment
image = imread('tb3_world.pgm');
imageCropped = image(120:250,135:265);
%imshow(imageCropped);

imageBW = imageCropped < 100;
%imshow(imageBW);

figure;
tb3map = binaryOccupancyMap(imageBW);


% Define parameters
vMap = validatorOccupancyMap;
vMap.Map = tb3map;

% Astar planner
planner = plannerHybridAStar(vMap, 'MinTurningRadius', 2.0);

% Set start and goal points
start = [35 35 -pi];
goal = [98 98 0];

% Plan the path
route = plan(planner, start, goal);
route = route.States;

% Get poses from the route.
rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
startPoses = route(1:end-1,:);
endPoses = route(2:end,:);

rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end

% Display the map
figure
show(planner)
title('A* path')