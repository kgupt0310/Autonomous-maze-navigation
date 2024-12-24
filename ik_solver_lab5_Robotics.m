robot = importrobot('/Users/kshitijgupta/Desktop/mycobot_pro_600_new/mycobot_pro_600.urdf');

% Visualize the robot (optional)
show(robot);

% Create an inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);

% Define the end-effector (tool) name
endEffector = 'link6';

% Define weights for the IK solver [x, y, z, roll, pitch, yaw]
weights = [1, 1, 1, 1, 1, 1];

% Define an array of desired coordinates
desiredCoordinates = [
    [-0.190036025, -0.4357352],
    [-0.190036025, -0.4014392],
    [-0.15617587, -0.4014392],
    [-0.12231571500000002, -0.4014392],
    [-0.12231571500000002, -0.3671432],
    [-0.12231571500000002, -0.3328472],
    [-0.12231571500000002, -0.2985512],
    [-0.12231571500000002, -0.26425519999999997],
    [-0.15617587, -0.26425519999999997],
    [-0.190036025, -0.26425519999999997],
    [-0.190036025, -0.2985512],
    [-0.190036025, -0.3328472],
    [-0.22389618000000003, -0.3328472],
    [-0.25775633500000006, -0.3328472],
    [-0.25775633500000006, -0.2985512],
    [-0.25775633500000006, -0.26425519999999997],
    [-0.25775633500000006, -0.22995919999999997],
    [-0.25775633500000006, -0.19566319999999995],
    [-0.25775633500000006, -0.16136719999999996]
]
;

% Fixed values for z, roll, pitch, and yaw
z = 0.06;
roll = 3.14;
pitch = -1.54;
yaw = -1.54;

% Convert Euler angles to a rotation matrix
rotm = eul2rotm([yaw, pitch, roll]);

% Initial guess for joint positions
initialGuess = homeConfiguration(robot);

for i = 1:size(desiredCoordinates, 1)
    x = desiredCoordinates(i, 1);
    y = desiredCoordinates(i, 2);
    
    % Combine rotation matrix and position into a homogeneous transformation matrix
    targetPose = trvec2tform([x, y, z]) * rotm2tform(rotm);
    
    % Solve IK
    [configSol, solInfo] = ik(endEffector, targetPose, weights, initialGuess);
    
    % Check if configSol is a valid structure and has JointPosition field
    if isstruct(configSol) && isfield(configSol, 'JointPosition')
        % Display the solution
        disp(['Solution for coordinate set ', num2str(i), ':']);
        
        disp(configSol(1))
        % % Check if JointPosition is valid and has the expected number of angles
        % if isempty(configSol.JointPosition)
        %     disp('No valid joint configuration found.');
        %     continue; % Skip to the next iteration if no solution is found
        % end
        
        % Display joint angles in radians
        fprintf('Joint Angles (radians): ');
        fprintf('%f ', configSol.JointPosition);
       
        fprintf('\n');
       
         % Preallocate array for joint angles in degrees
        % jointAnglesDegrees = zeros(1, length(configSol.JointPosition)); % Preallocate based on number of joints
        
        % Manually convert joint angles to degrees and store them individually
        % for j = 1:length(configSol.JointPosition)
        %     jointAnglesDegrees(j) = configSol.JointPosition(j) * (180 / pi);  % Convert each angle to degrees
        % end

        % Display joint angles in degrees
        % fprintf('Joint Angles (degrees): ');
        % fprintf('%f ', jointAnglesDegrees);
        % fprintf('\n');
        % disp('---');
        
        % Optionally visualize each solution
        show(robot, configSol);
        pause(1); % Pause to view each configuration
    else
        disp(['Invalid configuration solution for coordinate set ', num2str(i)]);
        if ~isstruct(configSol)
            disp('configSol is not a structure.');
        else
            disp('JointPosition field does not exist in configSol.');
        end
    end
end
