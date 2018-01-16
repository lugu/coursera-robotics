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
% FIXME: fine tune
M = 100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = [ repmat(pose(:,1), [1, M]); zeros(1, M) ];
size(P)

x_sigma = 2.6777e-04;
y_sigma = 3.1766e-04;
theta_sigma = 8.1771e-05;

min_score = 800;

[ max_map_y, max_map_x] = size(map)

for j = 1:N

    j
    P(4,:) = zeros(1, M);

    for i = 1:columns(P)

        P(1, i) = mvnrnd(P(1,i), x_sigma);
        P(2, i) = mvnrnd(P(2,i), y_sigma);
        P(3, i) = mvnrnd(P(3,i), theta_sigma);

        robotPosX = P(1, i) * resol + origin(1);
        robotPosY = P(2, i) * resol + origin(2);
        robotPosAngl = P(3, i);

        for range = 1:1081 % for each time,
            tetha = robotPosAngl + scanAngles(range);
            rotationMatrix = [ cos(tetha), sin(tetha); -sin(tetha), cos(tetha) ];

            localDistance = ranges(range, j) * resol;
            localOcclusion = [ localDistance; 0 ];

            orig = [ robotPosX; robotPosY ];
            occ = orig + rotationMatrix * localOcclusion;
            occ = floor(occ);
            if min(occ) > 0
                if occ(1) < max_map_x
                    if occ(2) < max_map_y
                        P(4, i) += map(occ(2),occ(1));
                    end
                end
            end
        end


     end

     [m, index] = max(P(4,:))
     pose(1, j) = P(1, index);
     pose(2, j) = P(2, index);
     pose(3, j) = P(3, index);

     score_filter = min(m, min_score);
     idx = ( P(4,:) >= score_filter);
     P = P(:,idx);
     missing = M - columns(P)
     idx = randi(columns(P), missing, 1);
     P = [ P P(:,idx) ];

% for j = 2:N % You will start estimating pose from j=2 using ranges(:,2).
%
%     % 1) Propagate the particles
%
%
%     % 2) Measurement Update
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
%
%     %   2-2) For each particle, calculate the correlation scores of the particles
%
%     %   2-3) Update the particle weights
%
%     %   2-4) Choose the best particle to update the pose
%
%     % 3) Resample if the effective number of particles is smaller than a threshold
%
%     % 4) Visualize the pose on the map as needed
%
%
% end

myPose = pose;
end

