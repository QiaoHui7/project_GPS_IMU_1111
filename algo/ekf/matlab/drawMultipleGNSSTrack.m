function drawMultipleGNSSTrack(varargin)
% draw multiple GNSS track
% input : 
%   1. plotLatLonTrajectory(lat1, lon1, lat2, lon2, ...)
%   2. plotLatLonTrajectory({'name1', lat1, lon1}, {'name2', lat2, lon2}, ...)
%     lat - latitude data
%     lon - longitude data
%     (name) - track name
%
% example:
%   lat1 = [39.9, 39.95, 40.0];
%   lon1 = [116.3, 116.4, 116.5];
%   lat2 = [39.8, 39.85, 39.9];
%   lon2 = [116.2, 116.3, 116.4];
%   plotLatLonTrajectory(lat1, lon1, lat2, lon2);
%     or
%   plotLatLonTrajectory({'track_alpha', lat1, lon1}, {'track_beta', lat2, lon2});

    if nargin ~= 0 && mod(nargin, 2) ~= 0 && ~iscell(varargin{1})
        error('invalid input format');
    end

    hold off;

    if iscell(varargin{1}) % format 1: {(name,) lat, lon}
        colors = lines(length(varargin));
        for i = 1:length(varargin)
            if length(varargin{i}) < 2
                error('invalid input format');
            end

            if length(varargin{i}) >= 3 % {name, lat, lon}
                name = varargin{i}{1};
                lat = varargin{i}{2};
                lon = varargin{i}{3};
            else % {lat, lon}
                name = ['track_', num2str(i)];
                lat = varargin{i}{1};
                lon = varargin{i}{2};
            end

            if length(lat) ~= length(lon)
                error('length of latitude data does not match longitude');
            end

            geoscatter(rad2deg(lat), rad2deg(lon), 25, colors(i,:), 'o','filled','DisplayName', name);
            geobasemap satellite;
            hold on;
        end
    else % format 2: lat1, lon1, lat2, lon2...
        colors = lines(length(varargin) / 2);
        trackNum = length(varargin) / 2;
        for i = 1:trackNum
            lat = varargin{2 * i - 1};
            lon = varargin{2 * i};

            if length(lat) ~= length(lon)
                error('length of latitude data does not match longitude');
            end

            hold on;
            geoscatter(rad2deg(lat), rad2deg(lon), 25, colors(i,:), 'o','filled','DisplayName', ['轨迹', num2str(i)]);
        end
    end

    title('GNSS Tracks');
    legend('Location', 'northeast');
end