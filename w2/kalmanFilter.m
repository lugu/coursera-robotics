function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0]';
        % FIXME: stat with a very large uncertainty about the model
        param.P = 10000 * eye(4);
        % param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    sigmaPos2 = 0.1
    sigmaVel2 = 0.1

    sigmaMeasurement = diag ( [ sigmaPos2 , sigmaPos2 , sigmaVel2 , sigmaVel2 ] );
    % sigma = diag ( [ σpx2 , σpy2 , σvx2 , σvy2 ] );

    dt = t - previous_t % FIXME: shall it be 330ms ?

    % transition matrix
    A = [ 1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1 ]
    % 4 by 4

    % C observation matrix: z = C * x
    C = [ 1, 0, 0, 0; 0, 1, 0, 0 ]
    % 2 by 4

    sigmaObs2 = 0.1
    sigmaObservation = diag ( [ sigmaObs2 , sigmaObs2 ] );
    % sigmaObservation = diag ( [ σzx2 , σzy2 ] );

    R = sigmaObservation % 2 by 2
    P = A * param.P * A' + sigmaMeasurement % 4 by 4
    K = P * C' * (inv(R + C * P * C'))
    % (4 by 4 * 4 by 2) * (2 by 2 + 2 by 4 * 4 by 4 * 4 by 2) => 4 by 2

    state = A * state
    % 4 by 4 * 4 by 1

    sigmaP = P - K * C * P
    % 4 by 4 - 4 by 2 * 2 by 4 * 4 by 4 => 4 by 4

    z = [ x, y, 0, 0 ]
    newX = A * state + K * ( z - C * A * state )
    % 4 by 4 * 4 by 1 + 4 by 2 * (4 by 1 - 2 by 4 * 4 by 1) => 4 by 1

    predictx = newX(1)
    predicty = newX(2)
    param.P = P

end
