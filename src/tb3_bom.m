% This code converts the .pgm image file of the environment obtained from 
% SLAM through ROS to a Binary Occupancy Map which is used as an input for
% navigation

image = imread('tb3_world.pgm');
imageCropped = image(120:250,135:265);

figure;
imshow(imageCropped)

figure;
imageBW = imageCropped < 100;
imshow(imageBW)

figure;
tb3map = binaryOccupancyMap(imageBW);
show(tb3map)