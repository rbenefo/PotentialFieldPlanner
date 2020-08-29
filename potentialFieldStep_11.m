function [qNext, isDone] = potentialFieldStep_11(qCurr, map, robot)
% POTENTIALFIELDSTEP_GROUPNO Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.

%%
%% Initialize 
persistent time %time variable stores the number of steps the planner has taken. If too many steps, the planner times out
if isempty(time) 
    time = 1;
else
    time = time+1;
end
persistent qLog
if isempty(qLog)
    qLog = zeros(1,6);
end
timeout = 1000; %the planner times out in 1000 steps
rConToPar = 30; %threshold for switching from conic potential well to parabolic potential well
zetas =  [0.005, 0.005, 0.01, 0.01, 0.01,0.01]; %attraction gains
rho0 = 200; %maximum distance for repulsive force to act
eta = [1000,1000,1000,1000,1000,2000]*100; %repulsion gains
alp = 0.01; %step size alpha
tolToGoal = 0.3; %distance needed to be from goal to declare that the planner has completed
epsilon = 0.005; %minimum step; if step falls below this threshold for an extended amount of time, we declare the robot in a local min
obj_list = map.obstacles; %pull obstacle list from map
currloc = calculateFK_sol(qCurr); %current location in workspace coords
netfr = zeros(1,3); %initialize force vector
goalloc = calculateFK_sol(map.goal); %goal location in workspace coords 
tau = zeros(1,6); %initialize torque matrix 
persistent v
if isempty(v)
    v = [0.01, 0.02, 0.02, 0.02, 0.02, 0.05]/10; %initialize standard_deviation vector (to calculate random walk step size)
end
%%
for i=1:6 %make a separate potential field for every joint
    dist_a = (currloc(i,:).' - goalloc(i,:).'); %calcualte distance in workspace coordaintes from current position to goal position
    if norm(dist_a) > rConToPar %if the robot is far away from the goal, use a conic attractive field
        Fa(i,:) = -dist_a/norm(dist_a); %Calculate conic attractive force
    else %else, use a parabolic attractive field
        Fa(i,:) = -zetas(i)*dist_a; %Calculate parabolic attractive force
    end
    for j = 1:size(obj_list,1) %for every obstacle
        [dist, vec] =  distPointToBox(currloc(i,:),obj_list(j,:));
        if dist > rho0 %if too far away from obstacle
            Fr(i,:) = zeros(1,3); %set repulsive force to 0
        else %if close enough to obstacle
            Fr(i,:) = -eta(i)*((1/dist - (1/rho0))*(1/dist^2)*vec/norm(vec)); %calculate repulsive force
        end
        netfr = netfr+Fr(i,:); %get net repulsive force (sum of repulsive forces from all obstacles)
    end
    J = calcJacobian_11(qCurr, i+1, robot); %get jacobian based on current configuration
    Jv = J(1:3, :); %extract Jv from J
    netf = netfr+Fa(i,:); %get net force (attractive + repulsive)
    taui = forceToTorque_11(netf, Jv); %calculate torque from force
    tau(i,:) = [taui.', zeros(1,6-length(taui))]; %if needed, add zeros to end of torque vector to make it 1x6, and append to torque matrix
end

taunet = sum(tau); %collapse torque matrix down to a vector (all 6 joint 1 torques are added, all 5 joint 2 torques, all 4 joint 3, etc)
qLog(time,:) = qCurr + alp*taunet/norm(taunet); %execute gradient descent


if (norm(qLog(time, :)-map.goal) > tolToGoal) %if we're at a distance (in config space) to the goal above the threshold (too far away)
    isDone = 0; %not done, continue planning
    if time > 5
        if mean([abs(qLog(time, :) - qLog(time -1, :)), abs(qLog(time, :)-qLog(time-2, :)), abs(qLog(time, :)-qLog(time-3, :))]) < epsilon
            isCollided = 1; %initialize 
            t = 6; %want a random walk of length 6 (6 steps)
            while isCollided == 1 %while the proposed random step intersects with an obstacle
                randarr =[normrnd(0, v(1)^2*t), normrnd(0, v(2)^2*t), normrnd(0, v(3)^2*t), normrnd(0, v(4)^2*t), normrnd(0, v(5)^2*t), normrnd(0, v(6)^2*t)]; %generate random array of stepsize pulled from distribution with standard deviations stored in v vector
                qProposed = qLog(time, :)+randarr;%propose random step
                isCollided = isRobotCollided(qProposed, map, robot); %check to see if random step causes robot to intersect with any obstacles
            end
            v = v*1.001; %gradually make the random walk more agressive if it doesn't get us out of the local minimum
            if any(v > 0.05) %make sure the random walk doesn't get too agressive (otherwise, the planner will cause the robot to execute large jumps)
                v = [0.01, 0.02, 0.02, 0.02, 0.02, 0.05];
            end
            qLog(time, :) = qProposed; %if the proposed random step doesn't intersect withn an obstacle, execute it
        end
    end
else %if we're close enough to the goal
    isDone = 1; %raise the isDone flag
    fprintf("Goal (more or less) reached!\n") %announce planner success
    fprintf("Distance between joints and goal positions:\n[")
    fprintf(' %g ', round(qLog(end,:)-map.goal, 4)) %print out the final distance to the goal
    fprintf("]\n")

end

qNext = qLog(end, :);
if time > timeout %if planning takes too long
    isDone = 1; %raise the isDone flag
    fprintf("Planner took too long to solve.\n") %announce planner failure
end


        

end