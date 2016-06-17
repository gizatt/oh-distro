options.floating = 'quat';
robot = RigidBodyManipulator('../../../perception/manipulation_tracking/urdf/irb140_chull_ati_gelsight.urdf', options);
    
lc = lcm.lcm.LCM.getSingleton();
lcmonitor_robot_state = drake.util.MessageMonitor(bot_core.robot_state_t,'utime');
lc.subscribe('EST_ROBOT_STATE', lcmonitor_robot_state);

lcmonitor_markers = drake.util.MessageMonitor(drake.lcmt_optotrak, 'timestamp');
lc.subscribe('irb140_gelsight_markers', lcmonitor_markers);

lcmonitor_save_trigger = drake.util.MessageMonitor(bot_core.utime_t,'utime');
lc.subscribe('IRB140_MARKER_FITTER_SAVE_TRIGGER', lcmonitor_save_trigger);

lcmonitor_optim_trigger = drake.util.MessageMonitor(bot_core.utime_t,'utime');
lc.subscribe('IRB140_MARKER_FITTER_OPTIM_TRIGGER', lcmonitor_optim_trigger);

last_trigger_optim_utime = 0;
last_trigger_save_utime = 0;
last_state_utime = -1;
last_markers_utime = -1;

% which markers are on which links?
marker_links = {{17, 'link_1'};
                {18, 'link_1'};
                {19, 'link_1'};
                {20, 'link_3'};
                {21, 'link_3'};
                {22, 'link_3'};
                {23, 'link_4'};
                {24, 'link_4'};
                {25, 'link_4'};
                {26, 'link_4'};
                {27, 'link_4'};
                {28, 'link_4'};
                {29, 'link_4'};
                {30, 'link_4'};
                {31, 'link_4'};
               };
marker_array = [];
for i=1:length(marker_links)
   marker_array = [marker_array; [marker_links{i}{1} robot.findLinkId(marker_links{i}{2})]];
end

x = zeros(6 + 3*length(marker_links), 1);

x = [0.6077
    2.0407
    0.9150
   -2.7010
    1.4807
   -1.2521
    0.0628
   -0.2397
    0.2102
    0.0651
   -0.2200
    0.2098
    0.0804
   -0.2299
    0.2103
    0.0138
    0.0021
    0.0944
   -0.0067
    0.0030
    0.0938
    0.0042
    0.0199
    0.0940
    0.1730
   -0.0444
    0.0699
    0.1531
   -0.0435
    0.0692
    0.1639
   -0.0264
    0.0695
    0.1733
   -0.0081
    0.0710
    0.1528
   -0.0066
    0.0707
    0.1643
    0.0103
    0.0707
    0.0442
    0.0103
    0.0726
    0.0413
   -0.0095
    0.0732
    0.0253
    0.0029
    0.0739];

% convenience for gettin gout to yaml
complete_yaml_string = '';
for marker_i = 1:length(marker_links)
    marker_id = marker_links{marker_i}{1};
    marker_linkname = marker_links{marker_i}{2};
    marker_pos = x((marker_i*3+4):(marker_i*3+6));
    complete_yaml_string = [complete_yaml_string sprintf('      - ids: [%d]\n        body: "%s"\n        pos: [%d, %d, %d]\n', ...
        marker_id, marker_linkname, marker_pos(1), marker_pos(2), marker_pos(3))];
end

q_history = {};
marker_history = {};
checkDependency('lcmgl');
lcmgl = drake.util.BotLCMGLClient(lc, 'irb140_marker_fitter3');

while true

    %% Update robot state
    state_raw = lcmonitor_robot_state.getMessage();
    if (~isempty(state_raw))
        robot_state = bot_core.robot_state_t(state_raw);
        
        q = double([robot_state.pose.translation.x;
             robot_state.pose.translation.y;
             robot_state.pose.translation.z;
             robot_state.pose.rotation.w;
             robot_state.pose.rotation.x;
             robot_state.pose.rotation.y;
             robot_state.pose.rotation.z;
             robot_state.joint_position;]);
         
        last_state_utime = robot_state.utime;
    end
    
    markers_raw = lcmonitor_markers.getMessage();
    if (~isempty(markers_raw))
       markersmsg =  drake.lcmt_optotrak(markers_raw);
       markers = double([markersmsg.x markersmsg.y markersmsg.z].');
       markers(markers < -20000) = 0;
       last_markers_utime = markersmsg.timestamp;
    end
    
    trigger_save_raw = lcmonitor_save_trigger.getMessage();
    if (~isempty(trigger_save_raw))
        trigger_save = bot_core.utime_t(trigger_save_raw);
        
        if (trigger_save.utime ~= last_trigger_save_utime && last_state_utime > 0 && last_markers_utime > 0)
            % store current state in history
            q_history = [q_history; q];
            marker_history = [marker_history; markers];
            last_trigger_save_utime = trigger_save.utime;
            'Saving.'
        end
    end
            
    trigger_optim_raw = lcmonitor_optim_trigger.getMessage();
    if (~isempty(trigger_optim_raw))
        trigger_optim = bot_core.utime_t(trigger_optim_raw);
        if (trigger_optim.utime ~= last_trigger_optim_utime && last_state_utime > 0 && last_markers_utime > 0)
            % set up optimization
            % we need to determine the camera transform
            % and the transform of each marker on its link
            N_markers = length(marker_links);
            nx = N_markers*3 + 6;
            x_name = {'camera_pose_x';
                      'camera_pose_y';
                      'camera_pose_z';
                      'camera_pose_roll';
                      'camera_pose_pitch';
                      'camera_pose_yaw'};

            for i=1:N_markers
               ind = marker_array(i, 1);
               x_name = [x_name;
                         {sprintf('marker_%d_x', ind);
                          sprintf('marker_%d_y', ind);
                          sprintf('marker_%d_z', ind)}];
            end
            prog = NonlinearProgram(nx, x_name);

            for i=1:length(marker_history)
                q = q_history{i};
                markers = marker_history{i};

                options_kinsol.use_mex = false;
                options_kinsol.compute_gradients = false;
                kinsol = doKinematics(robot, q, [], options_kinsol);

                % add cost object for every marker
                for marker_i = 1:length(marker_links)
                    fk = relativeTransform(robot, kinsol, 1, marker_array(marker_i, 2));
                    mk = markers(:, marker_array(marker_i, 1))/1000.0;
                    if (mk(3) < -0.5 && mk(3) > -10)
                        xind = [1:6, (marker_i*3+4):(marker_i*3+6)];
                        const = FunctionHandleConstraint(0, 0, 9, @(x)IRB140_marker_fitter_transform_constraint(x, fk, mk));
                        %const.grad_method = 'numerical';
                        prog = addCost(prog,const,xind);
                    end
                end
            end
            prog = prog.addConstraint(BoundingBoxConstraint(-5*ones(3, 1), 5*ones(3,1)), [1;2;3]);
            prog = prog.addConstraint(BoundingBoxConstraint(-pi*ones(3,1), pi*ones(3,1)), [4;5;6]);
            prog = prog.addConstraint(BoundingBoxConstraint(-0.5*ones(nx-6, 1), 0.5*ones(nx-6, 1)), [7:nx]);
            'Running optim...'
            prog = prog.setSolver('fmincon');
            prog = prog.setSolverOptions('fmincon', 'Algorithm', 'interior-point');
            prog = prog.setSolverOptions('fmincon', 'TolX', 0.00001);
            prog = prog.setSolverOptions('fmincon', 'MaxIter', 500);
            [x,objval,exitflag] = prog.solve(x)
            last_trigger_optim_utime = trigger_optim.utime;
        end
    end
    
    if (last_state_utime > 0 && last_markers_utime > 0)
        %% draw
        marker_points_in_world_frame = [];
        lcmgl.glColor3f(1, 0, 1);

        options_kinsol.use_mex = false;
        options_kinsol.compute_gradients = false;
        kinsol = doKinematics(robot, q, [], options_kinsol);
        for marker_i = 1:length(marker_links)
            fk = forwardKin(robot, kinsol, marker_array(marker_i, 2), x((marker_i*3+4):(marker_i*3+6)));
            %lcmgl.sphere(fk,0.05,20,20);
            marker_points_in_world_frame = [marker_points_in_world_frame fk];
        end
        lcmgl.points(marker_points_in_world_frame(1,:),marker_points_in_world_frame(2,:),marker_points_in_world_frame(3,:));
        lcmgl.switchBuffers;
    end

    
    pause(1/50);
end