function draw_ekf_data(dataPath, isSave, savePath)
H = ...
    [0.000260478118165929, 0.00165093517151693, 0.00457698035603714, 0.00855019113817552, 0.0114001411354267, 0.0104357028814206, 0.00468582898988947, -0.00327224795312655, -0.00820580691464230, -0.00608376501212253, 0.00187428829931122, 0.00900596105556488, 0.00827881839624281, -0.000976447630217112, -0.0110463455685493, -0.0117839087440022, -0.000410968923476379, 0.0139593529900202, 0.0169947988477819, 0.00278690721373362, -0.0179572251355613, -0.0249311515643641, -0.00693602068464902, 0.0239785072234191, 0.0384477474541343, 0.0149955257824083, -0.0352939036210979, -0.0682872625002372, -0.0365233340512236, 0.0710174231020678, 0.211377065375294, 0.310072115366234, 0.310072115366234, 0.211377065375294, 0.0710174231020678, -0.0365233340512236, -0.0682872625002372, -0.0352939036210979, 0.0149955257824083, 0.0384477474541343, 0.0239785072234191, -0.00693602068464902, -0.0249311515643641, -0.0179572251355613, 0.00278690721373362, 0.0169947988477819, 0.0139593529900202, -0.000410968923476379, -0.0117839087440022, -0.0110463455685493, -0.000976447630217112, 0.00827881839624281, 0.00900596105556488, 0.00187428829931122, -0.00608376501212253, -0.00820580691464230, -0.00327224795312655, 0.00468582898988947, 0.0104357028814206, 0.0114001411354267, 0.00855019113817552, 0.00457698035603714, 0.00165093517151693, 0.000260478118165929];

% read csv
df = readtable(dataPath);

% get GNSS raw data and filtered data
df_raw = df(:, 4:22);
df_filtered = df(:, 23:27);

% get bounding box
bbox = [min(df_raw.gps_lon), max(df_raw.gps_lon), ...
    min(df_raw.gps_lat), max(df_raw.gps_lat)];

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

% draw raw data
x_r = df_raw.gps_lon;
y_r = df_raw.gps_lat;
%yaw_r = deg2rad(df_raw.angle_z(1:5:end));

% raw data start and end point
plot(x_r(1), y_r(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
plot(x_r(end), y_r(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% raw data scatter
scatter(x_r, y_r, 30, 'b', 'filled', 'MarkerFaceAlpha', 0.8, ...
    'DisplayName', 'GNSS Raw data');

% raw data directional arraw
% quiver(x_r(1:5:end), y_r(1:5:end), cos(yaw_r), sin(yaw_r), 0.5, 'k', 'LineWidth', 0.5, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');

% draw filtered data
x_f = df_filtered.ekf_lon;
y_f = df_filtered.ekf_lat;
%yaw_f = deg2rad(df_filtered.ekf_heading(1:5:end));

% filtered data start and end point
plot(x_f(1), y_f(1), 'yo', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
plot(x_f(end), y_f(end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% filtered data scatter
scatter(x_f, y_f, 30, 'g', 'filled', 'MarkerFaceAlpha', 0.8, ...
    'DisplayName', 'EKF Filtered GNSS');

% filtered data directional arraw
% quiver(x_f(1:5:end), y_f(1:5:end), cos(yaw_f), sin(yaw_f), 0.5, 'k', 'LineWidth', 0.5, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');

% add title and legend
title('GNSS Raw (Vs) EKF Filtered GNSS');
legend('Location', 'northwest');
grid on;

if (isSave)
    saveas(gcf, strcat(savePath, '/raw_vs_ekf.svg'));
end


% X
figure;
plot(df_raw.acc_x);
title('acc-x');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-x.svg'));
end

% X fir-filtered
acc_xf = conv(df_raw.acc_x, H, "same");
figure;
plot(acc_xf);
title('acc-xf');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-xf.svg'));
end


% Y
figure;
plot(df_raw.acc_y);
title('acc-y');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-y.svg'));
end

% Y fir-filtered
acc_yf = conv(df_raw.acc_y, H, "same");
figure;
plot(acc_yf);
title('acc-yf');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-yf.svg'));
end


% acc Z
figure;
plot(df_raw.acc_z);
title('acc-z');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-z.svg'));
end

% Z fir-filtered
acc_zf = conv(df_raw.acc_z, H, "same");
figure;
plot(acc_zf);
title('acc-zf');
xlabel('sample index');
ylabel('g');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/acc-zf.svg'));
end

fprintf('mean of acc x/y/z: %.4f / %.4f / %.4f\n', ...
    mean(df_raw.acc_x), mean(df_raw.acc_y), mean(df_raw.acc_z));
fprintf('std of acc x/y/z: %.4f / %.4f / %.4f\n', ...
    std(df_raw.acc_x), std(df_raw.acc_y), std(df_raw.acc_z));

fprintf('mean of filtered acc x/y/z: %.4f / %.4f / %.4f\n', ...
    mean(acc_xf), mean(acc_yf), mean(acc_zf));
fprintf('std of filtered acc x/y/z: %.4f / %.4f / %.4f\n', ...
    std(acc_xf), std(acc_yf), std(acc_zf));

% spin around axis x
figure;
plot(deg2rad(df_raw.gyro_x));
title('spin-x');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-x.svg'));
end

% spin around axis x fir-filtered
gyro_xf = conv(deg2rad(df_raw.gyro_x), H, "same");
figure;
plot(gyro_xf);
title('spin-xf');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-xf.svg'));
end


% spin around axis y
figure;
plot(deg2rad(df_raw.gyro_y));
title('spin-y');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-y.svg'));
end

% spin around axis y fir-filtered
gyro_yf = conv(deg2rad(df_raw.gyro_y), H, "same");
figure;
plot(gyro_yf);
title('spin-yf');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-yf.svg'));
end


% spin around axis z
figure;
plot(deg2rad(df_raw.gyro_z));
title('spin-z');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-z.svg'));
end

% spin around axis z fir-filtered
gyro_zf = conv(deg2rad(df_raw.gyro_z), H, "same");
figure;
plot(gyro_zf);
title('spin-zf');
xlabel('sample index');
ylabel('rad/s');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/spin-zf.svg'));
end

fprintf('mean of gyro x/y/z: %.4f / %.4f / %.4f\n', ...
    mean(deg2rad(df_raw.gyro_x)), mean(deg2rad(df_raw.gyro_y)), mean(deg2rad(df_raw.gyro_z)));
fprintf('std of gyro x/y/z: %.4f / %.4f / %.4f\n', ...
    std(deg2rad(df_raw.gyro_x)), std(deg2rad(df_raw.gyro_y)), std(deg2rad(df_raw.gyro_z)));

fprintf('mean of filtered gyro x/y/z: %.4f / %.4f / %.4f\n', ...
    mean(gyro_xf), mean(gyro_yf), mean(gyro_zf));
fprintf('std of filtered gyro x/y/z: %.4f / %.4f / %.4f\n', ...
    std(gyro_xf), std(gyro_yf), std(gyro_zf));

% speed east
figure;
plot(df_raw.gps_kph_east);
title('speed-east');
xlabel('sample index');
ylabel('km/h');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/speed-east.svg'));
end


% speed north
figure;
plot(df_raw.gps_kph_north);
title('speed-north');
xlabel('sample index');
ylabel('km/h');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/speed-north.svg'));
end


% speed up
figure;
plot(df_raw.gps_kph_up);
title('speed-up');
xlabel('sample index');
ylabel('km/h');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/speed-up.svg'));
end


% magnetic x
figure;
plot(df_raw.mag_x);
title('magnetic-x');
xlabel('sample index');
ylabel('μT');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-x.svg'));
end


% magnetic y
figure;
plot(df_raw.mag_y);
title('magnetic-y');
xlabel('sample index');
ylabel('μT');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-y.svg'));
end


% magnetic z
figure;
plot(df_raw.mag_z);
title('magnetic-z');
xlabel('sample index');
ylabel('μT');
grid on;
if (isSave)
    saveas(gcf, strcat(savePath, '/magnetic-z.svg'));
end
end
