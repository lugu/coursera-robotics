% Robotics: Estimation and Learning
% WEEK 4
%
% Complete this function following the instruction.
function myPose = particleLocalization(ranges, scanAngles, map, param)
% myPose is a 3 by N matrix representing (x, y, theta) at each timestep.

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
pose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters
%
% % the number of grids for 1 meter.
resol = param.resol;
% % the origin of the map in pixels
origin = param.origin;

% The initial pose is given
pose(:,1) = param.init_pose;

% You should put the given initial pose into pose for j=1, ignoring the j=1 ranges.
% The pose(:,j) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = [ repmat(pose(:,1), [1, M]); zeros(1, M) ];
size(P)

x_std =  0.016364 * 2
y_std =  0.017823 * 2
theta_std =  0.0090428 * 2

min_score = 1200;

[ max_map_y, max_map_x] = size(map)

for j = 1:N

    j

    P(1,:) = P(1,:) + randn(1, M) * x_std;
    P(2,:) = P(2,:) + randn(1, M) * y_std;
    P(3,:) = P(3,:) + randn(1, M) * theta_std;
    P(4,:) = zeros(1, M);

    robotPosX = P(1, :) * resol + origin(1);
    % 1-by-M
    robotPosY = P(2, :) * resol + origin(2);
    % 1-by-M
    robotPosAngl = P(3, :);
    % 1-by-M

    occlusionsX = zeros(M, 1081);
    occlusionsY = zeros(M, 1081);

    for range = 1:1081 % for each time,
        tetha = robotPosAngl + scanAngles(range);
        % 1-by-M
        rotationMatrixTop = [ cos(tetha); sin(tetha) ];
        % 2-by-M
        rotationMatrixBottom = [ -sin(tetha); cos(tetha) ];
        % 2-by-M
        localDistance = ranges(range, j) * resol;
        % 1-by-1
        localOcclusion = [ localDistance; 0 ];
        % 2-by-1
        occX = rotationMatrixTop' * localOcclusion + robotPosX';
        % M-by-1 = m-by-2 * 2-by-1 + M-by-1
        occY = rotationMatrixBottom' * localOcclusion + robotPosY';
        % M-by-1 = m-by-2 * 2-by-1 + M-by-1
        occM = [ occX' ; occY' ];
        % 2-by-M
        occM = floor(occM);
        % 2-by-M

        occlusionsX(:,range) = occM(1,:);
        occlusionsY(:,range) = occM(2,:);
     end

     columnsP = size(P,2);
     for i = 1:columnsP
         occlusions = [ occlusionsX(i,:); occlusionsY(i,:) ];
         % 2-by-1081
         idx = ( occlusions(1,:) > 0);
         occlusions = occlusions(:,idx);
         idx = ( occlusions(1,:) < max_map_x);
         occlusions = occlusions(:,idx);

         idx = ( occlusions(2,:) > 0);
         occlusions = occlusions(:,idx);
         idx = ( occlusions(2,:) < max_map_y);
         occlusions = occlusions(:,idx);
         % 2-by-1081-or-less

         % create the logical indexes
         T = [max_map_y 1];
         idx = T * (occlusions - [0; 1]);
         P(4, i) = sum(map(idx));

     end

     [m, index] = max(P(4,:));
     m
     pose(1, j) = P(1, index);
     pose(2, j) = P(2, index);
     pose(3, j) = P(3, index);

     score_filter = min(m, min_score);
     idx = ( P(4,:) >= score_filter);
     P = P(:,idx);
     columnsP = size(P,2);
     missing = M - columnsP;
     idx = randi(columnsP, missing, 1);
     P = [ P P(:,idx) ];
  end


myPose = pose;
end

