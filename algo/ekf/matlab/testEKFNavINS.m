ekf = ekfNavINS();

dr = readtable('./test/walk_test/2025_11_11_17_00_sunny/data/ekf_2025_11_11_17_00_43.csv');

% 1 (latitude), 2 (longitude), 3 (altitude)
% 4 (velocity-north), 5 (velocity-east), 6 (velocity-down)
% 7 (roll), 8 (pitch), 9(yaw)
% 10 (groundtrack)
df = zeros(height(dr), 10);

for i = 1:height(dr)
    ekf.ekf_update(dr.timestamp(i), ...
        dr.gps_kph_north(i)/3.6, dr.gps_kph_east(i)/3.6, dr.gps_kph_up(i)/3.6, ...
        deg2rad(dr.gps_lat(i)), deg2rad(dr.gps_lon(i)), 0, ...
        deg2rad(dr.gyro_x(i)), deg2rad(dr.gyro_y(i)), deg2rad(dr.gyro_z(i)), ...
        dr.acc_x(i)*9.794, dr.acc_y(i)*9.794, dr.acc_z(i)*9.794, ...
        dr.mag_x(i)*1e-3, dr.mag_y(i)*1e-3, dr.mag_z(i)*1e-3);

    df(i, :) = [rad2deg(ekf.getLatitude_rad()), rad2deg(ekf.getLongitude_rad()), ekf.getAltitude_m(), ...
        ekf.getVelNorth_ms(), ekf.getVelEast_ms(), ekf.getVelDown_ms(), ...
        ekf.getRoll_rad(), ekf.getPitch_rad(), ekf.getHeadingConstrainAngle180_rad(), ...
        ekf.getGroundTrack_rad()];
end

% get bounding box
bbox = [min(df(:, 2)), max(df(:, 2)), ...
    min(df(:, 1)), max(df(:, 1))];

% create figure
figure('OuterPosition', [0, 0, 1080, 1080]);
hold on;

% set coordinate axis range
xlim([bbox(1), bbox(2)]);
ylim([bbox(3), bbox(4)]);

% format axis
ax = gca;
ax.XAxis.Exponent = 0;
ax.YAxis.Exponent = 0;
xtickformat('%.7f');
ytickformat('%.7f');

% draw Matlab ekf-filtered data
xm = df(:, 2);
ym = df(:, 1);

% Matlab ekf-filtered data start and end point
plot(xm(1), ym(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
plot(xm(end), ym(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% Matlab ekf-filtered data scatter
scatter(xm, ym, 30, 'b', 'filled', 'MarkerFaceAlpha', 0.8, ...
    'DisplayName', 'EKF Filtered GNSS by Matlab');

% draw C++ ekf-filtered data
xc = dr.ekf_lon;
yc = dr.ekf_lat;

% C++ ekf-filtered data start and end point
plot(xc(1), yc(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
plot(xc(end), yc(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% C++ ekf-filtered data scatter
scatter(xc, yc, 30, 'g', 'filled', 'MarkerFaceAlpha', 0.8, ...
    'DisplayName', 'EKF Filtered GNSS by C++');

title('GNSS EKF Filtered Matlab (Vs) C++');
legend('Location', 'northwest');
grid on;
