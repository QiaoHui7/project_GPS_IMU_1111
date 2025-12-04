clear; clc;

%% configure query parameter
road_name = "浦电路";
% bounding-box: [min_lat, min_lon, max_lat, max_lon]
bbox = [31.21778, 121.513889, 31.221111, 121.519444];
overpass_url = "https://overpass-api.de/api/interpreter";

%% construct Overpass QL query string
query = sprintf(['[out:json];\n' ...
                 '(\n' ...
                 '  way["name"="%s"](%f,%f,%f,%f);\n' ...
                 '  >;\n' ...
                 ');\n' ...
                 'out body;'], ...
                 road_name, bbox(1), bbox(2), bbox(3), bbox(4));

%% send POST request
options = weboptions('Timeout', 30, ...
                     'HeaderFields', {'User-Agent', 'MATLABOverpass/1.0 (cnjstzjychen@gmail.com)'}, ...
                     'ContentType', 'text', ...
                     'CharacterEncoding', 'UTF-8');

try
    response = webwrite(overpass_url, 'data', query, options);
    data = jsondecode(response);
    % if isstruct(data)
    %     disp('data is struct');
    %     elements = data.elements;
    %     if isstruct(elements)
    %         disp('elements is struct');
    %     elseif iscell(elements)
    %         disp('elements is cell');
    %         element = elements{1};
    %         if isstruct(element)
    %             disp('element is struct');
    %             disp(element);
    %         elseif iscell(element)
    %             disp('element is cell');
    %         end
    %     end
    % elseif iscell(data)
    %     disp('data is cell');
    % end
catch ME
    fprintf('request failed: %s\n', ME.message);
    return;
end

%% parse JSON to extract coordinates of nodes and roads
% ID -> [lat, lon]
node_map = containers.Map('KeyType', 'int64', 'ValueType', 'any');

% extract all nodes
node_count = 0;
for i = 1:length(data.elements)
    element = data.elements{i};
    if isfield(element, 'type') && strcmp(element.type, 'node')
        if isfield(element, 'id') && isfield(element, 'lat') && isfield(element, 'lon')
            node_map(element.id) = [element.lat, element.lon];
            node_count = node_count + 1;
        else
            fprintf("WARNING: node %d lack 'id' or 'lat' or 'lon'\n", node_id);
        end
    end
end

fprintf('successfully extract coordinates of %d node\n\n', node_count);

%% extract roads segments
road_segments = {}; % save nodes lat/lon of each road
way_info = {};  % save information of each road
way_count = 0;

for i = 1:length(data.elements)
    element = data.elements{i};

    % "type": "way"
    if isfield(element, 'type') && strcmp(element.type, 'way') && isfield(element, 'id')

        is_target_road = false;

        % check if road name consistent
        if isfield(element, 'tags')
            if isfield(element.tags, 'name') && strcmp(element.tags.name, road_name)
                is_target_road = true;
            elseif isfield(element.tags, 'name:zh') && strcmp(element.tags.('name:zh'), road_name)
                is_target_road = true;
            end
        end

        if is_target_road
            way_count = way_count + 1;
            nodes = [];
            missing_nodes = [];

            % traverse all nodes IDs of the road to match lat/lon
            for j = 1:length(element.nodes)
                node_id = element.nodes(j);
                if isKey(node_map, node_id)
                    coord = node_map(node_id);
                    nodes = [nodes; [coord, node_id]];
                else
                    missing_nodes = [missing_nodes; node_id];
                end
            end

            % save all information of the road
            if ~isempty(nodes)
                % segment_info.id = element.id;
                % segment_info.coordinates = coordinates;
                % segment_info.node_count = size(coordinates, 1);
                % segment_info.missing_nodes = missing_nodes;
                % segment_info.tags = element.tags;

                road_segments{end + 1} = {element.id, nodes};
                % way_info{end+1} = segment_info;

                fprintf('found road %d (ID: %d) consisting of %d nodes', ...
                    way_count, element.id, size(nodes, 1));

                if ~isempty(missing_nodes)
                    fprintf(' (miss %d nodes)', length(missing_nodes));
                end
                fprintf('\n');
            end
        end
    end
end

%% save result as .csv
if isempty(road_segments)
    fprintf('none road segment found on %s！\n', road_name);
else
    fprintf('successfully extract %d road segments on "%s"\n\n', length(road_segments), road_name);

    drawArgs = {};
    % save nodes lat/lon of each road segment
    for i = 1:length(road_segments)
        road_segment = road_segments{i};
        road_id = road_segment{1};
        road_nodes = road_segment{2};

        csv_filename = sprintf('%s_segment%d_way%d.csv', road_name, i, road_id);
        T = table(road_nodes(:, 1), road_nodes(:, 2), road_nodes(:, 3),...
                  'VariableNames', {'latitude', 'longitude', 'node_id'});
        writetable(T, csv_filename, 'WriteVariableNames', true);
        fprintf('  successfully save %s\n\n', csv_filename);

        drawArgs{end + 1} = {['way', int2str(road_id)], deg2rad(road_nodes(:, 1)), deg2rad(road_nodes(:, 2))};
    end

    drawMultipleGNSSTrack(drawArgs{:});

    % % save all segments data
    % if length(road_segments) > 1
    %     all_coordinates = [];
    %     all_segment_ids = [];
    % 
    %     for i = 1:length(road_segments)
    %         segment = road_segments{i};
    %         segment_ids = repmat(i, size(segment, 1), 1);
    %         way_ids = repmat(way_info{i}.id, size(segment, 1), 1);
    % 
    %         all_coordinates = [all_coordinates; segment]; %#ok<AGROW>
    %         all_segment_ids = [all_segment_ids; segment_ids]; %#ok<AGROW>
    %     end
    % 
    %     csv_filename = sprintf('%s_全路段_汇总.csv', road_name);
    %     T = table(all_segment_ids, all_coordinates(:, 1), all_coordinates(:, 2), ...
    %               'VariableNames', {'segment_id', 'latitude', 'longitude'});
    %     writetable(T, csv_filename, 'WriteVariableNames', true);
    %     fprintf('成功保存所有路段数据为: %s\n', csv_filename);
    % end
end

% %% 可视化道路段
% if ~isempty(road_segments)
%     figure('Name', '浦东南路道路坐标可视化', 'NumberTitle', 'off', ...
%            'Position', [100, 100, 800, 600]);
%     hold on;
% 
%     % 使用不同的颜色和标记
%     colors = lines(length(road_segments));
% 
%     for i = 1:length(road_segments)
%         segment = road_segments{i};
%         info = way_info{i};
% 
%         % 绘制道路线
%         plot(segment(:, 2), segment(:, 1), ...
%              'Color', colors(i, :), 'LineWidth', 2.5, ...
%              'DisplayName', sprintf('路段 %d (Way %d)', i, info.id));
% 
%         % 绘制节点
%         scatter(segment(:, 2), segment(:, 1), ...
%                 50, colors(i, :), 'filled', 'MarkerEdgeColor', 'k', ...
%                 'DisplayName', sprintf('路段 %d 节点', i));
%     end
% 
%     % 添加图例和标签
%     xlabel('经度', 'FontSize', 12);
%     ylabel('纬度', 'FontSize', 12);
%     title(sprintf('"%s"道路坐标分布 (%d个路段)', road_name, length(road_segments)), ...
%           'FontSize', 14, 'FontWeight', 'bold');
% 
%     % 创建两个图例：一个用于线段，一个用于点
%     hLines = findobj(gca, 'Type', 'line');
%     hScatter = findobj(gca, 'Type', 'scatter');
% 
%     if ~isempty(hLines)
%         legend(hLines(1:length(road_segments)), 'Location', 'best');
%     end
% 
%     grid on;
%     axis equal;
% 
%     % 添加比例尺
%     xlims = xlim;
%     ylims = ylim;
%     text(xlims(1) + 0.01*(xlims(2)-xlims(1)), ...
%          ylims(1) + 0.05*(ylims(2)-ylims(1)), ...
%          sprintf('总节点数: %d', size(all_coordinates, 1)), ...
%          'FontSize', 10, 'BackgroundColor', 'white');
% 
%     hold off;
% 
%     % 保存可视化图像
%     saveas(gcf, sprintf('%s_道路可视化.png', road_name));
%     fprintf('\n已保存可视化图像: %s_道路可视化.png\n', road_name);
% end
