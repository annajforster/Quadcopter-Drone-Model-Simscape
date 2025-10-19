function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(path_number,varargin)
%quadcopter_select_trajectory Obtain parameters for selected quadcopter trajectory
%   [waypoints, timespot_spl, spline_data, spline_yaw] = quadcopter_select_trajectory(path_number)
%   This function returns the essential parameters that define the
%   quadcopter's trajectory. The function returns
%
%       waypoints       Key x-y-z locations the quadcopter will pass through
%       timespot_spl    Times the quadcopter will pass through points along
%                       the spline that defines its path
%       spline_data     Points used for interpolating the spline that
%                       defines the path of the quadcopter
%       spline_yaw      Yaw angle at the spline_data points

% Copyright 2021-2024 The MathWorks, Inc.

if(nargin == 2)
    roundtrip = varargin{1};
else
    roundtrip = false;
end

switch (path_number)
    case 1
        waypoints = [ ...
            -2    -2  -2  -2  -2  -2  -2  -2   -2
            -2    -2  -2  -2  -2  -2  -2  -2   -2
            0.14   6   6   6   6   6   6   6   0.14];
        spline_data = waypoints';
        timespot_spl = [0 12 15 18 22 24 28 32 50]';
        spline_yaw = [0 0 0 0 0 0 0 0 0];
        % max_speed = 1;
        % min_speed = 0.1;
        % xApproach = [4 0.5];
        % vApproach = 0.1;

    % case 2
    %     waypoints = [ ...
    %         -2    -2  -2  -2  -2  -2  -2  -2  -2   -2
    %         -2    -2  -2  -2  -2  -2  -2  -2  -2   -2
    %         0.14   6   6   6   4   4   4   2   2   0.14];
    %     spline_data = waypoints';
    %     timespot_spl = [0 25 27 31 35 37 39 42 45 60]';
    %     spline_yaw = [0 0 0 0 0 0 0 0 0 0];

case 2
    % ---- Desired profile (times in seconds) ----
    t0   = 0;    z0 = 0.14;   % pad height
    t_up = 15;   z1 = 6.00;   % climb to 6 m in 15 s
    h1   = 8;                 % hover at 6 m
    t_d1 = 6;    z2 = 4.00;   % descend to 4 m in 6 s
    h2   = 8;                 % hover at 4 m
    t_d2 = 6;    z3 = 2.00;   % descend to 2 m in 6 s
    h3   = 8;                 % hover at 2 m
    t_ld = 8;    zf = 0.14;   % land in 8 s
    g    = 0.25;              % guard (s) to clamp dz/dt≈0 at hovers

    % Absolute times
    t1s = t0 + t_up;           % reach 6 m
    t1e = t1s + h1;            % end 6 m hover
    t2e = t1e + t_d1;          % reach 4 m
    t2h = t2e + h2;            % end 4 m hover
    t3e = t2h + t_d2;          % reach 2 m
    t3h = t3e + h3;            % end 2 m hover
    tfe = t3h + t_ld;          % touch down

    % COLUMN vector (N×1) — use semicolons to force rows explicitly
    timespot_spl = [ ...
        t0;
        t1s-g; t1s; t1s+g;
        t1e-g; t1e; t1e+g;
        t2e-g; t2e; t2e+g;
        t2h-g; t2h; t2h+g;
        t3e-g; t3e; t3e+g;
        t3h-g; t3h; t3h+g;
        tfe ];

    % Z levels (N×1) — exact same length/order as timespot_spl
    z = [ ...
        z0;
        z1; z1; z1;
        z1; z1; z1;
        z2; z2; z2;
        z2; z2; z2;
        z3; z3; z3;
        z3; z3; z3;
        zf ];

    % X,Y held constant (N×1)
    N = numel(timespot_spl);
    x = -2 * ones(N,1);
    y = -2 * ones(N,1);

    % Shapes to match your downstream code:
    % waypoints: 3×N, spline_data: N×3, spline_yaw: 1×N
    waypoints   = [x y z]';        % 3×N
    spline_data = waypoints';         % N×3
    spline_yaw  = zeros(1,N);      % 1×N


case 3
    % -------- Altitude & timing (seconds) --------
    z0 = 0.14; z1 = 1.00; zf = 0.14;
    t0 = 0;
    t_up = 5;           % gentle climb to 1 m
    h1  = 10;           % first hover
    t_ff = 10;          % forward flight duration (time-based)
    h2  = 10;           % second hover
    t_ld = 7;           % gentle landing
    g = 0.35;           % guards to clamp slopes at hover edges

    % -------- XY limits & start --------
    X_LIMITS = [-2, 12.5];
    Y_LIMITS = [-2,  8.5];

    x0 = -2;  y0 = -2;                % given start
    y0 = min(max(y0, Y_LIMITS(1)), Y_LIMITS(2));

    % Time-based forward distance: slow & stable
    v_ff = 0.25;                       % m/s (change here to go further)
    D    = v_ff * t_ff;                % metres travelled during forward leg
    x1   = min(x0 + D, X_LIMITS(2));   % clamp to X limit

    % -------- Absolute times --------
    t1s = t0 + t_up;            % reach 1 m
    t1e = t1s + h1;             % end hover 1
    t2e = t1e + t_ff;           % end forward flight (time-based)
    t2h = t2e + h2;             % end hover 2
    tfe = t2h + t_ld;           % touchdown

    % -------- Time knots (column) --------
    Nramp = 17;                                 % dense S-curve samples
    ramp_times = linspace(t1e+g, t2e-g, Nramp).';

    timespot_spl = [ ...
        t0;
        t1s-g; t1s; t1s+g;                      % clamp at 1 m
        t1e-g; t1e; t1e+g;                      % start of forward leg
        ramp_times(2:end-1);                    % interior ramp samples
        t2e-g; t2e; t2e+g;                      % end of forward leg
        t2h-g; t2h; t2h+g;                      % clamp at 1 m
        tfe ];

    N = numel(timespot_spl);

    % -------- Position vectors --------
    x = x0 * ones(N,1);
    y = y0 * ones(N,1);
    z = z0 * ones(N,1);

    % Altitude: hold at 1 m across both hovers (with guards), land at end
    z((timespot_spl >= (t1s-g)) & (timespot_spl <= (t2h+g))) = z1;
    z(end) = zf;

    % -------- Smoothstep S-curve in X over [t1e+g, t2e-g] --------
    mask = (timespot_spl >= (t1e+g)) & (timespot_spl <= (t2e-g));
    idx  = find(mask);
    if numel(idx) >= 3
        t_start = timespot_spl(idx(1));
        t_end   = timespot_spl(idx(end));
        t_r = linspace(t_start, t_end, Nramp).';
        u   = (t_r - t_start) ./ max(t_end - t_start, eps);   % 0..1
        s   = 3*u.^2 - 2*u.^3;                                % smoothstep
        x_r = x0 + (x1 - x0) * s;

        % write interior points; endpoints pinned by guards
        x(idx(2:end-1)) = x_r(2:end-1);
        x(idx(1))       = x0;
        x(idx(end))     = x1;
    end

    % Y stays constant; clamp just in case
    y = min(max(y, Y_LIMITS(1)), Y_LIMITS(2));

    % -------- Output shapes --------
    waypoints   = [x y z]';     % 3×N (as your function expects)
    spline_data = waypoints';      % N×3
    spline_yaw  = zeros(1,N);   % keep yaw fixed




    case 4
        waypoints = [...
            -3.0000    0.5633    4.5492    7.7662    9.0011    7.3491    3.7145   -0.0156    2.2687    5.0000
            -5.0000   -4.4724   -4.5758   -2.3910    1.5272    5.3013    6.5986    6.8774    9.5797    8.0000
            0.1500    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    0.15];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 5
        waypoints = [ ...
            0    0 50  50 100  100 150 150 150
            0    0  0  50  50  100 100 150 150
            0.15 6  6   6   6    6   6   6 0.14];
        max_speed = 2;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;
        
    case 6
        waypoints = [ ...
            0    0 150 150 150
            0    0 0   150 150
            0.15 6 6   6   0.14];
        max_speed = 2;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;
end

% Only call the function to calculate target speed and yaw angle if needed
% Paths that define the spline data and yaw angles explictly should not
% define parameter xApproach
if(exist("xApproach","var"))
    if(roundtrip)
        [timespot_spl_re, spline_data_re, spline_yaw_re, ~] = ...
            quadcopter_waypoints_to_trajectory(...
            fliplr(waypoints),max_speed,min_speed,xApproach,vApproach);

        [timespot_spl_to, spline_data_to, spline_yaw_to, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
        
        pause_at_target = 5; % sec
        timespot_spl = [timespot_spl_to; timespot_spl_re+timespot_spl_to(end)+pause_at_target];

        spline_data = [spline_data_to;spline_data_re];
        spline_yaw = [spline_yaw_to spline_yaw_re];
        spline_yaw = unwrap(spline_yaw,1.5*pi);
    else
        [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
    end
else
    % Obtain data to visualize path between waypoints
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    if(roundtrip)
        spline_data  = [spline_data; flipud(spline_data)];
        %timespot_spl
        timespot_spl = [timespot_spl; timespot_spl(end)+5; ...
            timespot_spl(end)+5+cumsum(flipud(diff(timespot_spl)))];
        spline_yaw = unwrap([spline_yaw flipud(spline_yaw)+pi],1.5*pi);
    end
end
