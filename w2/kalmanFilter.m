function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    % dt = 330ms ?
    % transition matrix A = [ 1, dt ; 0, 1 ]
    A = [ 1, 0.033 ; 0 , 1 ];

    % sigma = diag ( [ σpx2 , σpy2 , σvx2 , σvy2 ] );

    % C observation matrix: z = C * x
    C = [ 1, 0, 0, 0 ; 0, 1, 0, 0 ];

    % sigmaMeasurement = diag ( [ σzx2 , σzy2 ] );

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / (t - previous_t);
    vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    predictx = x + vx * 0.330;
    predicty = y + vy * 0.330;
    % State is a four dimensional element
    state = [x, y, vx, vy];
end
