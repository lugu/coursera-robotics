% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 

function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

% Compute if a given Hue value is with
% 80% of the pre-computed model
function y = Threshold(x)
    sigma2 =    5.22885680941758e-04;
    MU =  0.161095484991759;
    thre = sqrt(sigma2) * 1.5;
    if (x < (MU - thre)) 
        y = 0;
    else
        if (x > (MU + thre))
            y = 0;
        else
            y = 255;
        end
    end
end


% Use Hue component only with the simplest gaussian model
HSV = rgb2hsv(I);
H = HSV(:,:,1);
segI = arrayfun(@(x) Threshold(x), H);

% select only the largest region
CC = bwconncomp(segI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
BW(CC.PixelIdxList{idx}) = 0;
Z = zeros(size(segI));
Z(CC.PixelIdxList{idx}) = 255;
segI = Z;

% dillate and erode to smooth the values
SE = strel('arbitrary',eye(7));
segI = imdilate(segI,SE);
segI = imerode(segI,SE);

% extract the center of the region
s = regionprops(segI,'centroid');
centroids = cat(1, s.Centroid);
loc = centroids([end],:);

end
