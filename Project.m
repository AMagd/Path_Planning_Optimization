clear all
close all
clc
% run('bt-1.3/make_oct.m')

%% Options for plotting polytopes:
red.color = [1 0 0];
red.alpha = 1;

gray.color = [0.05 0.05 0.05];
gray.alpha = 0.01;

green.color = [0 1 0];
green.alpha = 0.1;

yellow.color = [1 1 0];
yellow.alpha = 0.2;
%% Generate random environment
%% 1. Random bounding box
rpx = rand*100; % center of bounding box in x
rpy = rand*100; % center of bounding box in y
rpz = rand*100; % center of bounding box in z

% creating bounding box randomly
margin = 15; % margin put to ensure no obstacles on the start and goal points
xlimits = [rpx-50*rand-50 rpx+50*rand+50];
ylimits = [rpy-50*rand-50 rpy+50*rand+50];
zlimits = [rpz-50*rand-50 rpz+50*rand+50];
[bounding_box_x, bounding_box_y, bounding_box_z] = meshgrid(xlimits, ylimits, zlimits);

fake_margin=0.1; % margin used to plot the bounding box such that it is not on top of some polytope (asthetics reasons only)
xlimits_fake = [xlimits(1)-fake_margin xlimits(2)+fake_margin];
ylimits_fake = [ylimits(1)-fake_margin ylimits(2)+fake_margin];
zlimits_fake = [zlimits(1)-fake_margin zlimits(2)+fake_margin];
[bounding_box_x_fake, bounding_box_y_fake, bounding_box_z_fake] = meshgrid(xlimits_fake, ylimits_fake, zlimits_fake);

bounding_box.V = [reshape(bounding_box_x,1,[]); reshape(bounding_box_y,1,[]); reshape(bounding_box_z,1,[])];
bounding_box_fake.V = [reshape(bounding_box_x_fake,1,[]); reshape(bounding_box_y_fake,1,[]); reshape(bounding_box_z_fake,1,[])];

%% Start and Goal Points:
start = [xlimits(1)+0.5*rand*(margin), ylimits(1)+0.5*rand*(margin), zlimits(1)+0.5*rand*(margin)];
goal = [xlimits(2)-0.5*rand*(margin), ylimits(2)-0.5*rand*(margin), zlimits(2)-0.5*rand*(margin)];
scatter3(start(1), start(2), start(3), 50, 'y', 'filled'); hold on
scatter3(goal(1), goal(2), goal(3), 50, 'r', 'filled'); hold on

plot(polyh(bounding_box_fake,'v'), gray); hold on

%% 2. Random obstacles
xlimits_obstacles_region = [xlimits(1)+margin xlimits(2)-margin];
ylimits_obstacles_region = [ylimits(1)+margin ylimits(2)-margin];
zlimits_obstacles_region = [zlimits(1)+margin zlimits(2)-margin];

nGrids = randi([1, 5]); % number of grids per axis
xsteps = (xlimits_obstacles_region(2)-xlimits_obstacles_region(1))/nGrids;
ysteps = (ylimits_obstacles_region(2)-ylimits_obstacles_region(1))/nGrids;
zsteps = (zlimits_obstacles_region(2)-zlimits_obstacles_region(1))/nGrids;

nObstacles = randi(min(20, nGrids^3)); % limit the number of obstacles to 10 obstacles
GridIndices = randi([1, nGrids], 3, nObstacles);
Obstacles_As = {};
Obstacles_bs = {};
Obstacles=[];
nPoints = randi([20, 30]); % number of points to construct an obstacle
for i=1:nObstacles
    Obstacle_ix = (xlimits_obstacles_region(1)+0.5*xsteps + xsteps*(GridIndices(1,i)-1)) + 0.495*(2*rand(1, nPoints)-1)*xsteps;
    Obstacle_iy = (ylimits_obstacles_region(1)+0.5*ysteps + ysteps*(GridIndices(2,i)-1)) + 0.495*(2*rand(1, nPoints)-1)*ysteps;
    Obstacle_iz = (zlimits_obstacles_region(1)+0.5*zsteps + zsteps*(GridIndices(3,i)-1)) + 0.495*(2*rand(1, nPoints)-1)*zsteps;
    Obstacles(:,:,end+1) = [Obstacle_ix; Obstacle_iy; Obstacle_iz];
    rep.V = Obstacles(:,:,end);
    plot(polyh(rep, 'v'), red); hold on
end

%% Generate collision-free polytopes:
[A_bounds, b_bounds] = vert2con(bounding_box.V');
nSeeds = nObstacles; % limit the number of obstacles to 10 obstacles
GridIndices = randi([0, nGrids-1], 3, nSeeds);
As = {};
bs = {};
seeds = [];
seeds_vec = [0.5 0.5 0];
safe_regions = struct('A', {}, 'b', {}, 'C', {}, 'd', {}, 'point', {});
for i=1:nSeeds
    randomized = seeds_vec(randperm(3));
    seed_ix = (xlimits_obstacles_region(1)+randomized(1)*xsteps+xsteps*(GridIndices(1,i)));
    seed_iy = (ylimits_obstacles_region(1)+randomized(2)*ysteps+ysteps*(GridIndices(2,i)));
    seed_iz = (zlimits_obstacles_region(1)+randomized(3)*zsteps+zsteps*(GridIndices(3,i)));
    seeds(end+1,:) = [seed_ix, seed_iy, seed_iz];
    [A, b, C, d, ~] = iris.inflate_region(Obstacles, A_bounds, b_bounds, seeds(end,:)');
    safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', seeds(end,:));
    polytope.B = A; polytope.b = b;
    plot(polyh(polytope, 'h'), green); hold on
    As{end+1}=A;
    bs{end+1}=b;
end

[A_start, b_start, C, d, ~] = iris.inflate_region(Obstacles, A_bounds, b_bounds, start');
safe_regions(end+1) = struct('A', A_start, 'b', b_start, 'C', C, 'd', d, 'point', start);
polytope.B = A_start; polytope.b = b_start;
plot(polyh(polytope, 'h'), green); hold on
As{end+1}=A_start;
bs{end+1}=b_start;

[A_goal, b_goal, C, d, ~] = iris.inflate_region(Obstacles, A_bounds, b_bounds, goal');
safe_regions(end+1) = struct('A', A_goal, 'b', b_goal, 'C', C, 'd', d, 'point', goal);
polytope.B = A_goal; polytope.b = b_goal;
plot(polyh(polytope, 'h'), green); hold on
As{end+1}=A_goal;
bs{end+1}=b_goal;

%% Path planning (Solving the optimization problem):
number_of_steps = 15; 
weight_goal = 1000;
weigth_inbetween = 0.1;
bigM = 100;
nPolytopes = length(bs);

cvx_solver mosek
cvx_begin
    variable x(3, number_of_steps)
    binary variable c(nPolytopes, number_of_steps);

    cost = 0;
    for i = 1:(number_of_steps-1)
        cost = cost + pow_pos(norm(x(:, i) - x(:, i+1), 1), 2)*weigth_inbetween;
    end
    cost = cost + pow_pos(norm(x(:, 1) - start'), 2)*weight_goal;
    cost = cost + pow_pos(norm(x(:, number_of_steps) - goal'), 2)*weight_goal;
    start_mat = diag(start);
    goal_mat = diag(goal);
    minimize( cost )
    subject to
        for i = 1:number_of_steps
            A_bounds*x(:, i) <= b_bounds;
            cs=0;
            for j = 1:nPolytopes
                cell2mat(As(j))*x(:, i) <= cell2mat(bs(j)) + (1-c(j, i))*bigM;
                cs = cs + c(j, i);
            end
            cs == 1;
        end
        
cvx_end
path = [start', x, goal'];
plot3(path(1, :), path(2, :), path(3, :),'o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
fnplt(cscvn(path),'b',1)

%% Plot variables:
title('Random Environment');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
legend('START', 'GOAL')
grid off