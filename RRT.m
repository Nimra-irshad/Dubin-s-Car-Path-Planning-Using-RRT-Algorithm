clear; clc;

close(findobj('type','figure','name','RRT basic'));
close(findobj('type','figure','name','RRT growing'));

% define the area and boundry
height = 10;
width = 10;
center = [0, 0];

% define the starting point and iterations
origin = [5,4];
iterations = 100;

offset = center - [width, height]./2;
vertecies = origin;
vert_count = 1;
edges.x = zeros(iterations,2);
edges.y = zeros(iterations,2);
ind_nearest = zeros(iterations,1);
edge_count = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
tic;
for i=1:iterations
    % random point generation
    x_rand = width*rand() + offset(1);
    y_rand = height*rand() + offset(2);
    
    % connect to nearest point
    ind_nearest(i) = dsearchn(vertecies, [x_rand, y_rand]);
    % edge_rand = [x_rand, y_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility
    
    % connect and add to list
    vertecies(vert_count+1,:) = [x_rand, y_rand];
    vert_count = vert_count + 1;
    edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
    edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
    edge_count = edge_count + 1;
    
    % plot animation here is undoable probably due to MATLAB running
    % optimization...
    % scatter(x_rand, y_rand, 10,'filled'); hold on;
    % plot([vertecies(ind_nearest,1); x_rand], [vertecies(ind_nearest,2); y_rand]); hold on;

end
toc;
clear i x_rand y_rand edge_rand

figure('name', 'RRT basic');
scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
scatter(vertecies(:,1), vertecies(:,2), 10,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
plot(edges.x', edges.y');
title('RRT Algorithm')
axis equal