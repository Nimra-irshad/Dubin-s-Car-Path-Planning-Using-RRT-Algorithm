%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close(findobj('type','figure','name','RRT w/ Dubins curve'));
close(findobj('type','figure','name','RRT growing'));
clear;

global route route1
% Planning area initialization
height = 10;
width = 10;
center = [0 0];
% Initial condition of [x, y, direction]
origin = [-4,-4, 20*pi/180];

% final vertex
goal = [4, 4, 20*pi/180];

turning_rad = 0.3;      % dubins turning raduis
% Define iterations count
iteration = 100;

offset = center - [width, height]./2;
th_range = 2*pi;    % do not change without knowledge about Dubins
th_center = 0;
th_offset = 0;

% prelocation of data for Origin
edges.x = zeros(iteration,2);
edges.y = zeros(iteration,2);
edges.th = zeros(iteration,2);
edges.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges.param(iteration).seg_param = [0, 0, 0];   % the lengths of the three segments
edges.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges.param(iteration).flag = 0;

% prelocation of data for Goal
edges1.x = zeros(iteration,2);
edges1.y = zeros(iteration,2);
edges1.th = zeros(iteration,2);
edges1.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges1.param(iteration).seg_param = [0, 0, 0];   % the lengths of the three segments
edges1.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges1.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges1.param(iteration).flag = 0;


vertecies = origin;
vert_count = 1;
ind_nearest = zeros(iteration,1);
edge_count = 0;

vertecies1 = goal;
vert_count1 = 1;
ind_nearest1 = zeros(iteration,1);
edge_count1 = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
tic;
for i=1:iteration
    % random point generation
    x_rand = width*rand() + offset(1);
    y_rand = height*rand() + offset(2);
    th_rand = th_range*rand() + th_offset;
    
    % connect to nearest point
    [ind_nearest(i),param_nearest] = dubins_searchn(vertecies, [x_rand, y_rand, th_rand], turning_rad);
    % edge_rand = [x_rand, y_rand, th_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest.flag < 0)
        % goto next loop
        %i = i-1; %doesn't work under MATLAB
    else
        % append the newly generated point and edge to the existing list
        vertecies(vert_count+1,:) = [x_rand, y_rand, th_rand];
        vert_count = vert_count + 1;

        edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
        edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
        edges.th(edge_count+1,:) = [vertecies(ind_nearest(i),3), th_rand];
        edges.param(edge_count+1) = param_nearest;
        edge_count = edge_count + 1;
    end
    
    % plot animation here is undoable probably due to MATLAB running
    % optimization...
    % scatter(x_rand, y_rand, 10,'filled'); hold on;
    % plot([vertecies(ind_nearest(i),1); x_rand], [vertecies(ind_nearest(i),2); y_rand]); hold on;
    
    x1_rand = width*rand() + offset(1);
    y1_rand = height*rand() + offset(2);
    th1_rand = th_range*rand() + th_offset;
    
    % connect to nearest point
    [ind_nearest1(i),param_nearest1] = dubins_searchn(vertecies1, [x1_rand, y1_rand, th1_rand], turning_rad);
    % edge_rand = [x_rand, y_rand, th_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest1.flag < 0)
        % goto next loop
        %i = i-1; %doesn't work under MATLAB
    else
        % append the newly generated point and edge to the existing list
        vertecies1(vert_count1+1,:) = [x1_rand, y1_rand, th1_rand];
        vert_count1 = vert_count1 + 1;

        edges1.x(edge_count1+1,:) = [vertecies1(ind_nearest1(i),1), x1_rand];
        edges1.y(edge_count1+1,:) = [vertecies1(ind_nearest1(i),2), y1_rand];
        edges1.th(edge_count1+1,:) = [vertecies1(ind_nearest1(i),3), th1_rand];
        edges1.param(edge_count1+1) = param_nearest1;
        edge_count1 = edge_count1 + 1;
    end
    
end
toc;
clear i x_rand y_rand edge_rand th_rand param_nearest

figure('name', 'RRT w/ Dubins curve');
plot_RRT_Dubins(gca, vertecies, edges, vert_count,vertecies1, edges1, vert_count1);
connect_path = [route(1:2,:) fliplr(route1(1:2,:))];
hold on; plot(gca, connect_path(1,:), connect_path(2,:), '--k','Linewidth',1); hold on

% plot bounderies
boundary = [offset; offset+[width 0]; offset+[width height]; offset+[0 height]; offset];
plot(boundary(:,1),boundary(:,2),'--r','Linewidth',3);