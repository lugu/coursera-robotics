% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
pkg load image
close all

imagepath = './train';

for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));

    [ maskedImage, loc ] = detectBall(I);
    imshow(maskedImage)
    hold on
    plot(loc(:,1),loc(:,2), 'b*')
    hold off
    pause
end
