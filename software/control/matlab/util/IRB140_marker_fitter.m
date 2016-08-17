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

x = [0.6166
    2.0375
    0.9059
   -2.9075
    1.4964
   -1.4656
    0.0640
   -0.2512
    0.2143
    0.0658
   -0.2313
    0.2138
    0.0824
   -0.2429
    0.2144
    0.0121
   -0.0045
    0.0960
   -0.0084
   -0.0041
    0.0956
    0.0021
    0.0130
    0.0958
    0.1720
   -0.0427
    0.0685
    0.1519
   -0.0423
    0.0682
    0.1623
   -0.0249
    0.0685
    0.1715
   -0.0064
    0.0700
    0.1509
   -0.0054
    0.0699
    0.1621
    0.0118
    0.0699
    0.0421
    0.0092
    0.0724
    0.0394
   -0.0107
    0.0729
    0.0233
    0.0016
    0.0737];

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
            prog = prog.setSolverOptions('fmincon', 'TolX', 0.0000001);
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