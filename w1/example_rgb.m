% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
pkg load image
close all

imagepath = './train';
maskpath = './mask';
Samples = [];
SamplesImage = [];
for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    SamplesImage = [SamplesImage; [R G B]];
    
    % % Collect samples 
    % disp('');
    % disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    % disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    % figure(1), 
    % % mask = roipoly(I); 
    maskI = imread(sprintf('%s/%03d.png',maskpath,k)) > 0;
    mask = maskI(:,:,1); % only one component
    sample_ind = find(mask);

    % figure(2), imshow(I .* maskI); title('Segment');
    % pause

    sample_ind = find(mask > 0);
    % disp('sample_ind: ' + sample_ind)
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
end

colors = double(Samples) ./ 256;
% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),30, colors)
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%

