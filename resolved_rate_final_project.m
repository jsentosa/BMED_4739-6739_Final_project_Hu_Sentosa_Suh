% Resolved-rates control for Q3_HW4
clear;
close all

% Setup parameters, the parameters are defined by expert, aka, yourself.
% you need to tune these parameters to meet your own requirement, such as
% how fast it will converge, and how accurate the robot will be 
tol_ori = [3e-2,0.5];  % [termination, brake point], rad
tol_pos = [1e-3,3e-2];  % [termination, brake point], m
v_max = 1; % max end effector linear velocity, m/s
w_max = 0.7; % max end effector angular velocity, rad/s
v_min = 0.07; % min end effector linear velocity, m/s
w_min = 0.14; % min end effector angular velocity, rad/s
singval_threshold = 1e-3; % threshold of triggering singularity robust pseudo-inverse
lambda = 1e-3; % singular value damping coefficient for singularity robustness
dt = 1e-2; % control cycle duration, s
iter = 1000; % max number of iterations for each target

% D-H parameters of the robot
params = [[15 0 0 0]' ... % d, m
          [0 10 10 10]' ... % a, m
          [pi/2 0 0 0]']; % alpha, rad

pen_len = 5; % length of the pen


history = []; % [joint values; end effector pose errors; singularity flag; target number]

% Setup for movie
fig = gcf;
axis_handle = fig.CurrentAxes;
set(gcf,'CurrentAxes',axis_handle);
loop_counter = 0;
plotting_skips = 1;

vidObj1 = VideoWriter('Q3_HW3_resolved_rate','MPEG-4');
open(vidObj1);
set(gcf,'renderer','zbuffer'); %MUST have this for getframe to work well under windows7
set(gca,'nextplot','replacechildren');


% setup for the visualization 
visualize = true; 
if visualize
    % initialize the figure for animation
    figure(1);
    view(3)
    axis equal
    grid on; hold on
    fig = gcf;
    axis_handle = fig.CurrentAxes;
    set(gcf,'CurrentAxes',axis_handle);
    set(gcf,'units','normalized','position',[0 0.05 0.5 0.75]);
end

q = [0.6021 0.9479 -2.3755 0]'; % initial joint values

% Initialize for a circular trajectory, you can change to the linear line
% or square or any other shape you like
N = 72; % number of targets
center = [20;20;10];
R = R_x(pi/3); % orientation of the circle
r = 5; % radius
Tt = zeros(4,4,N);
for i = 1:N
    alpha = i/N*2*pi;
    ca = cos(alpha); sa = sin(alpha);
    Tt(1,4,i) = R(1,1)*r*ca + R(1,2)*r*sa + center(1);
    Tt(2,4,i) = R(2,1)*r*ca + R(2,2)*r*sa + center(2);
    Tt(3,4,i) = R(3,1)*r*ca + R(3,2)*r*sa + center(3);
    Tt(1:3,1:3,i) = R;
end

% iteration starts here
for j = 1:size(Tt,3) % loop through all targets
    exitflag = 0;
    singular = false;
    for k = 1:iter + 1 % solve for each target
        % update the current end effector pose
        frames = Robot_forward_kinematics(q, params,pen_len);
        Tc = frames(:, :, end);

        % calculate end effector pose errors
        err_pos = Tt(1:3,4,j) - Tc(1:3,4);
        [err_ax, err_ang] = get_axis_angle(Tt(1:3,1:3,j)*Tc(1:3,1:3)');
        norm_err_pos = norm(err_pos);

        % Record each timea figure is created to the frame structure
        make_movbie1 = 0;
        if make_movbie1==1
            currFrame(size(Tt,3)) = getframe(gcf);
            writeVideo(vidObj1,currFrame(size(Tt,3)));
        end

        if visualize && (mod(k,5) == 1)
            cla(axis_handle);
            ht = draw_coordinates(Tt(1:3,4), Tt(1:3,1:3), 15e-2, 2); % draw target pose
            h = draw_Robot(frames);
            scatter3(squeeze(Tt(1,4,:)),squeeze(Tt(2,4,:)),squeeze(Tt(3,4,:)));
            % real-time data display
            str{1}=datestr(now);
            str{2}=['Target Pose # ',num2str(j),'   iteration #: ',num2str(k)];
            str{3}=['Current: ',...
                '  \theta_1=',num2str(q(1)),...
                '  \theta_2=',num2str(q(2)),...
                '  \theta_3=',num2str(q(3))];
            str{4}=['  \theta_4=',num2str(q(4))];
            str{5}=['Error Pos=',num2str(norm_err_pos),];
            disp = text(0.15, -0.15, -0.050, str, 'FontSize', 16);
            drawnow; % immediately draw the current configuration
        end

        % conditions for termination 
        if norm_err_pos < tol_pos(1)
            exitflag = 1;  % reached target within tolerance
            break % jump to the next iteration of the outer loop
        elseif k == iter+1 
            exitflag = 2;  % max iteration number exceeded
            break
        end

        % desired linear velocity
        % when close to target, slow down to prevent chattering
        if norm_err_pos < tol_pos(2)
            step_v = (v_max - v_min)*norm_err_pos/tol_pos(2) + v_min;
        else
            step_v = v_max;
        end

        % assemble the desired end effector twist
        v = step_v*err_pos/norm_err_pos;
        x_dot = v;

        % map to joint velocity
        J = Robot_Jacobian(frames); % calculate the robot jacobian

        % this will allow you to check whether the robot is in singular
        % position or not. if yes, we need to use the singularity robust
        % pseudo inverse, as shown below
        min_singval_J = min(svd(J));
        if min_singval_J < singval_threshold
            singular = true;
            pinvJ = J'/(J*J' + lambda*eye(size(J,1))); % singularity robust pseudo-inverse
            if visualize && (mod(k,5) == 1)
                disp = [disp text(frames(1,4,end),frames(2,4,end),frames(3,4,end)+0.02,'singular','FontSize',14)];
                drawnow; % immediately draw the current configuration
            end
        else
            singular = false;
            pinvJ = pinv(J);
        end

        % this is the basic jacobian equation, x_dot = J * q_dot
        q_dot = pinvJ*x_dot;

        % update joint values
        % In practice here we have to consider joint limits and joint velocity limits
        q = q + q_dot*dt;

        % record for further analysis
        history = [history [q; norm_err_pos; singular; j]];
    end
end

figure(2);
n = size(history,2);
yyaxis left
plot(1:n,history(5,:),'LineWidth',2);
ylabel('position error (m)');
yyaxis right
xlabel('iteration #');
grid on

if make_movbie1==1
    close(vidObj1);
end 

%% subfunctions
function frames = Robot_forward_kinematics(q, params,pen_len)
% Calculate the forward kinematics of Puma560 using the D-H convention
% q - 4x1 joint vector
% params - 6x3 D-H parameters [d, a, alpha]
% frames - 4x4x7 D-H frames and tool frame w.r.t the spatial frame

frames = zeros(4, 4, 5);
frames(:,:,1) = DH_transform(q(1), params(1,1), params(1,2), params(1,3));
for i = 2:4
    frames(:,:,i) = frames(:,:,i - 1)*DH_transform(q(i), params(i,1), params(i,2), params(i,3));
end
T_gripper2pen = [eye(3) [pen_len;0;0]; 0 0 0 1];
frames(:,:,5) = frames(:,:,4)*T_gripper2pen;
end

function T = DH_transform(theta, d, a, alpha)
% Calculate the homogeneous transformation matrix between adjacent D-H frames

st = sin(theta); ct = cos(theta);
sa = sin(alpha); ca = cos(alpha);
T = [ct  -st*ca   st*sa  a*ct;
     st   ct*ca  -ct*sa  a*st;
     0    sa      ca     d;
     0    0       0      1];
end

function J = Robot_Jacobian(frames)
% Calculate the Jacobian matrix of the puma 560, for velocities of the pen
% frames - 4x4x6 D-H frames w.r.t the spatial frame

% rotation = frames(1:3,1:3,:);  % rotation matrices of D-H frames
origin = frames(1:3,4,:);      % positions of origins of D-H frames
z = frames(1:3,3,:);  % the z-axis of D-H frames, aligned with joint axis

% linear velocity
Jv1 = cross([0;0;1], origin(:,:,4));
Jv2 = cross(z(:,:,1), (origin(:,:,4)-origin(:,:,1)));
Jv3 = cross(z(:,:,2), (origin(:,:,4)-origin(:,:,2)));
Jv4 = cross(z(:,:,3), (origin(:,:,4)-origin(:,:,3)));
Jv = [Jv1 Jv2 Jv3 Jv4];
% angular velocity 
J = Jv;
end

function [axis, angle] = get_axis_angle(R)
% Get the rotation axis and angle from a rotation matrix.
% make sure that R belongs to SO(3) or there can be numerical issues
% axis - 3x1 unit vector

angle = acos(min((trace(R)-1)/2,1));
if abs(angle-pi) < 1e-6
    axis = sqrt([R(1,1)+1; R(2,2)+1; R(3,3)+1]/2);
elseif angle < 1e-6
    axis = [0;0;1]; % almost no rotation so axis does not matter
else
    axis = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(angle));
end
end

function h = draw_Robot(frames)
% Draw the robot
h1 = draw_coordinates([0;0;0], eye(3), 15e-2, 2); % draw base frame
h2 = draw_coordinates(frames(1:3,4,end), frames(1:3,1:3,end), 15e-2, 2); % draw end effector frame
color = 'rgbmcyk';
linewidth = [7 6 5 4 3 2 1];
hl = line([0 frames(1,4,1)], [0 frames(2,4,1)], [0 frames(3,4,1)], ...
        'LineWidth', linewidth(1), 'Color', color(1));
h = [h1; h2; hl];
for i = 2:5
    hl = line([frames(1,4,i-1) frames(1,4,i)], [frames(2,4,i-1) frames(2,4,i)], [frames(3,4,i-1) frames(3,4,i)], ...
        'LineWidth', linewidth(i), 'Color', color(i));
    h = [h; hl];
end
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
end

function arrows = draw_coordinates(p,R,length,width)
% Draw Cartesian coordinates
% length - length of the axis
% width - line width of the axis

R=length.*R; %introduce length to the orthogonal vectors
hold on
arrow_x = quiver3(p(1),p(2),p(3),R(1,1),R(2,1),R(3,1),'r','filled','LineWidth',width);   % x axis
arrow_y = quiver3(p(1),p(2),p(3),R(1,2),R(2,2),R(3,2),'g','filled','LineWidth',width);   % y axis
arrow_z = quiver3(p(1),p(2),p(3),R(1,3),R(2,3),R(3,3),'b','filled','LineWidth',width);   % z axis
arrows = [arrow_x; arrow_y; arrow_z];
end