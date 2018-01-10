% Robotics: Estimation and Learning
% WEEK 3
%
% Complete this function following the instruction.
function myMap = occGridMapping(ranges, scanAngles, pose, param)
% Description of the variables: ranges, scanAngles, t, pose
% [1] ranges is 1081-by-K lidar sensor readings.
%     e.g. ranges(:,k) is the lidar measurement (in meter) at time index k.
% [2] scanAngles is 1081-by-1 array containing at what angles (in radian) the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [3] pose is 3-by-K array containing the pose of the mobile robot over time.
%     e.g. pose(:,k) is the [x(meter),y(meter),theta(in radian)] at time index k.


% map orientation:
%       +-------> x
%       |
%       |
%       V
%       y

% Find grids hit by the rays (in the gird map coordinate)
% Find occupied-measurement cells and free-measurement cells
% Update the log-odds
% Saturate the log-odd values
% Visualize the map as needed


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters
%
% % the number of grids for 1 meter.
resol = param.resol;

% % the initial map size in pixels
map = zeros(param.size);
% % the origin of the map in pixels
origin = param.origin;
%
% 4. Log-odd parameters
lo_occ = param.lo_occ;
lo_free = param.lo_free;
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,

      robotPosX = pose(1, j) * resol + origin(1);
      robotPosY = pose(2, j) * resol + origin(2);
      robotPosAngl = pose(3, j);

      for range = 1:1081 % for each time,

          tetha = robotPosAngl + scanAngles(range);

          rotationMatrix = [ cos(tetha), sin(tetha); -sin(tetha), cos(tetha) ];

          localDistance = ranges(range, j) * resol;
          localOcclusion = [ localDistance; 0 ];

          orig = [ robotPosX; robotPosY ];
          occ = orig + rotationMatrix * localOcclusion;
          occ = floor(occ);

          [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));

          free = sub2ind(size(map),freey(1:end-1),freex(1:end-1));

          % set occ cell values
          map(occ(2),occ(1)) = max(min(map(occ(2),occ(1)) + lo_occ, lo_max), lo_min);
          % set free cell values
          map(free) = max(min(map(free) - lo_free, lo_max), lo_min);

      end

end

myMap = map

end

