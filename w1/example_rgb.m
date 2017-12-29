% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
pkg load image
close all

imagepath = './train';
maskpath = './mask';
Samples = [];
SamplesRGB = [];

for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    HSV = rgb2hsv(I);
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
        
    H = HSV(:,:,1);
    S = HSV(:,:,2);
    V = HSV(:,:,3);
    
    maskI = imread(sprintf('%s/%03d.png',maskpath,k)) > 0;
    mask = maskI(:,:,1); % only one component
    sample_ind = find(mask);

    figure(2), imshow(I .* maskI); title('Segment');
    pause

    sample_ind = find(mask > 0);
    % disp('sample_ind: ' + sample_ind)
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);

    H = H(sample_ind);
    S = S(sample_ind);
    V = V(sample_ind);
    
    SamplesRGB = [SamplesRGB; [R G B]];
    Samples = [Samples; [H S V]];

end

format long

H = Samples(:,1);
N = size(H)(1);
MU = sum(H) / N
sigma2 = sum((H .- MU).^2) / N

% to compute the probability:
% (exp ( - (((x - MU) ^ 2) / (2 * sigma2)) )) / (sqrt(2 * pi * sigma2));

colors = double(SamplesRGB) ./ 256;
% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),30, colors)
title('Pixel Color Distribubtion');
xlabel('Hue');
ylabel('Saturation');
zlabel('Value');
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%
