clear; clc; close all;

global route route1
main_fig_name = 'RRT with dubins curve and obstacle detection';
close(findobj('type','figure','name',main_fig_name));

%%%%%%%%%%%%%%%%%%%% User Configuration Area %%%%%%%%%%%%%%%%%%%%
% define the map
map.height = 20;
map.width = 20;
map.center = [0, 0];
map.offset = map.center - [map.width, map.height]./2; % bottom-left corner

% starting vertex
origin = [-8, -6, 20*pi/180];

% final vertex
goal = [9, 8, 20*pi/180];


% define the obstacles using polygones, should be a cell stack of arrays.
poly = { [-3,-3; -1.5, -4; 0, -2.5; -0.5, -1; -3, 0],...
         [0,3; 3,3; 3, 6; 4, 6; 1.5, 8; -1, 6; 0, 6] };

% RRT parameters
iteration = 10;
th_range = 2*pi;    % do not change without knowledge about Dubins
th_center = 0;
th_offset = 0;

% dubins parameters
turning_rad = 0.5;
exp = 0.2;

%%%%%%%%%%%%%%%%%%%% End of configuration area %%%%%%%%%%%%%%%%%%%%
% main program starts here
poly = poly_verify(poly, map);
if ~iscell(poly)
    error('Ending due to incorrect obsticle polygon setting');
    return;
end

% Prelocation of data for Origin
edges.x = zeros(iteration,2);
edges.y = zeros(iteration,2);
edges.th = zeros(iteration,2);
edges.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges.param(iteration).seg_param = [0, 0, 0];   % the lengths of the three segments
edges.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges.param(iteration).flag = 0;

% Prelocation of data for Goal
edges1.x = zeros(iteration,2);
edges1.y = zeros(iteration,2);
edges1.th = zeros(iteration,2);
edges1.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges1.param(iteration).seg_param = [0, 0, 0];   % the lengths of the three segments
edges1.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges1.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges1.param(iteration).flag = 0;


% initial conditions, DO NOT CHANGE
vertecies = origin;
vertecies1 = goal;
vert_count = 1;
vert_count1 = 1;
ind_nearest = zeros(iteration,1);
ind_nearest1 = zeros(iteration,1);
edge_count = 0;
edge_count1 = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
% tic;
for i=1:iteration
    % random point generation
    x_rand = map.width*rand() + map.offset(1);
    y_rand = map.height*rand() + map.offset(2);
    th_rand = th_range*rand() + th_offset;
    
    % check if (x,y) is available or not
    if chk_xy_available([x_rand,y_rand], map, poly) == 0
        continue;
    end
        
    % connect to nearest point
    [ind_nearest(i),param_nearest] = dubins_searchn(vertecies, [x_rand, y_rand, th_rand], turning_rad);
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest.flag < 0)
        continue;  % goto next loop, reset i doesn't work in MATLAB
    elseif( chk_dubins_collision(param_nearest, map, poly, exp)==0 )
        %%%%%%%%%%%%%%%%% edit line %%%%%%%%%%%%%%%%
        % append the newly generated point and edge to the existing list
        vertecies(vert_count+1,:) = [x_rand, y_rand, th_rand];
        vert_count = vert_count + 1;

        edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
        edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
        edges.th(edge_count+1,:) = [vertecies(ind_nearest(i),3), th_rand];
        edges.param(edge_count+1) = param_nearest;
        edge_count = edge_count + 1;
%         plot_RRT_Dubins(gca, vertecies, edges, vert_count);
    end
    
    % random point generation
    x1_rand = map.width*rand() + map.offset(1);
    y1_rand = map.height*rand() + map.offset(2);
    th1_rand = th_range*rand() + th_offset;
    
    % check if (x,y) is available or not
    if chk_xy_available([x1_rand,y1_rand], map, poly) == 0
        continue;
    end
        
    % connect to nearest point
    [ind_nearest1(i),param_nearest1] = dubins_searchn(vertecies1, [x1_rand, y1_rand, th1_rand], turning_rad);
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest1.flag < 0)
        continue;  % goto next loop, reset i doesn't work in MATLAB
    elseif( chk_dubins_collision(param_nearest1, map, poly, exp)==0 )
        %%%%%%%%%%%%%%%%% edit line %%%%%%%%%%%%%%%%
        % append the newly generated point and edge to the existing list
        vertecies1(vert_count1+1,:) = [x1_rand, y1_rand, th1_rand];
        vert_count1 = vert_count1 + 1;

        edges1.x(edge_count1+1,:) = [vertecies1(ind_nearest1(i),1), x1_rand];
        edges1.y(edge_count1+1,:) = [vertecies1(ind_nearest1(i),2), y1_rand];
        edges1.th(edge_count1+1,:) = [vertecies1(ind_nearest1(i),3), th1_rand];
        edges1.param(edge_count1+1) = param_nearest1;
        edge_count1 = edge_count1 + 1;
%         plot_RRT_Dubins(gca, vertecies, edges, vert_count);
    end
    
end

% toc;
clear i x_rand y_rand th_rand param_nearest 


figure('name', main_fig_name);
% plot map
dubin;
boundary = [map.offset; map.offset+[map.width 0]; map.offset+[map.width, map.height]; map.offset+[0 map.height]; map.offset];
plot(boundary(:,1),boundary(:,2),'--r', 'Linewidth',3); hold on;
% plot obstacles
plot_obstacle_poly(gca, poly);
% plot path
plot_RRT_Dubins(gca, vertecies, edges, vert_count,vertecies1, edges1, vert_count1);

row = find(route == origin(1));
row1 = find(route1 == goal(1));

m = length(row);
n = length(row1);

total_path = [];
for k = 1:min(m,n);
    if (m==n)
        path = [route(row(k):row(k)+1,:) fliplr(route1(row1(k):row1(k)+1,:))];
        hold on; plot(gca, path(1,:), path(2,:), '--b','Linewidth',1); hold on
        total_path = [total_path; path];
    else if (m>n)
            path = [route(row(k):row(k)+1,:) fliplr(route1(row1(1):row1(1)+1,:))];
            hold on; plot(gca, path(1,:), path(2,:), '--b','Linewidth',1); hold on
            total_path = [total_path; path];
            else if (m<n)
                path = [route(row(1):row(1)+1,:) fliplr(route1(row1(k):row1(k)+1,:))];
                hold on; plot(gca, path(1,:), path(2,:), '--b','Linewidth',1); hold on
                total_path = [total_path; path];
                else
                    path = [route(row(1):row(1)+1,:) fliplr(route1(row1(1):row1(1)+1,:))];
                    hold on; plot(gca, path(1,:), path(2,:), '--b','Linewidth',1); hold on
                    total_path = [total_path; path];
                end
        end
    end
end

Short_dist = [];
standard_dv = [];
for h = 1:2:size(total_path)
    Dist = norm(total_path(h:h+1,:));
    SD = std(total_path(h:h+1,:)');
    Short_dist = [Short_dist; Dist];
    standard_dv = [standard_dv; SD];
end
[Y,I] = min(Short_dist);
if (I==1)
    hold on; plot(gca, total_path(I,:), total_path(I+1,:), '--k','Linewidth',1); hold on
else if (I==2)
        hold on; plot(gca, total_path(I+1,:), total_path(I+2,:), '--r','Linewidth',1); hold on
    else if (I==3)
            hold on; plot(gca, total_path(I+2,:), total_path(I+3,:), '--m','Linewidth',1); hold on
        end
    end
end
xlim([map.offset(1)-1,map.offset(1)+map.width+1]);
ylim([map.offset(2)-1,map.offset(2)+map.height+1]);

% figure(2);
% bar(standard_dv)
% title('Standard Deviation of Shortest Paths')

% figure(3);
% plot(boundary(:,1),boundary(:,2),'--r', 'Linewidth',3); hold on;
% plot_obstacle_poly(gca, poly); hold on;
% plot(gca, total_path(1,:), total_path(2,:), 'k','Linewidth',1); hold on
% plot(vertecies(1,1), vertecies(1,2),'mp','LineWidth',3); hold on;
% plot(vertecies1(1,1), vertecies1(1,2),'gp','LineWidth',3); hold on;
% title('Shortest Possible Path')
