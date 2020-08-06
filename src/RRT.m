% Load the occupancy map of the environment
image = imread('tb3_world.pgm');
imageCropped = image(120:250,135:265);
%imshow(imageCropped);

imageBW = imageCropped < 100;
%imshow(imageBW);

figure;
tb3map = binaryOccupancyMap(imageBW);
show(tb3map)


% Define start and goal points on the map
start = [35.0, 35.0, -pi];
goal = [98.0, 98.0, 0];

% Set the start and goal positions of the robot
hold on
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')

hold off

bounds = [tb3map.XWorldLimits; tb3map.YWorldLimits; [-pi pi]];

% Specify the state space of the robot and the min turning radius
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 2;

% Vaidate states and discretize motion
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = tb3map;
stateValidator.ValidationDistance = 0.05;

% Create a path planner and set parameters
planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 20000;


%planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

% Plan a path
[pthObj, solnInfo] = plan(planner, start, goal);

% Display the map
show(tb3map)
hold on

% Search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

% Interpolate and plot path
interpolate(pthObj,300)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

% Show the start and goal in the grid map
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')
title('RRT Path')
hold off


% function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
%     isReached = false;
%     threshold = 0.1;
%     if planner.StateSpace.distance(newState, goalState) < threshold
%         isReached = true;
%     end
% end