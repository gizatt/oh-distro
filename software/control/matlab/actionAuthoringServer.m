function actionAuthoringServer(IK)
% IK = 1 means we only require an IK solution
% IK = 2 meas that the we also do ZMP planning
if(nargin<1)
    action_options.IK = true;
    action_options.ZMP = false;
elseif(IK == 1)
    action_options.IK = true;
    action_options.ZMP = false;
elseif(IK == 2)
    action_options.IK = false;
    action_options.ZMP = true;
end
% listens for drc_action_sequence_t messages and, upon receipt, computes the
% IK and publishes the robot_state_t

lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');

% construct lcm input monitor
monitor = drake.util.MessageMonitor(drc.action_sequence_t(),'utime');
lc.subscribe('action_authoring_plan_action_request',monitor);

% construct lcm state publisher
% todo: should really load model name from lcm

s=warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator('../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', struct('floating','true'));
warning(s);
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(RobotStateCoder('atlas', joint_names)));
robot_plan_publisher =  RobotPlanPublisher('atlas',joint_names,true, ...
                                           'ACTION_AUTHORING_ROBOT_PLAN');
%%

% load the "zero position"
load('data/aa_atlas_fp.mat');
nq = r.getNumDOF();
q = xstar(1:nq);

% setup IK prefs
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q;

contact_tol = 1e-4;
v = r.constructVisualizer();

timeout=10;
while (1)
  data = getNextMessage(monitor,timeout);
  if ~isempty(data)
    msg = drc.action_sequence_t(data);
    action_sequence = ActionSequence();
    if(action_options.ZMP)
        action_sequence_ZMP = ActionSequence();
    end
    q_bk = q;
    % Get initial conditions from msg.q0
    msg.q0.robot_name = 'atlas'; % To match robot_state_coder.lcmcoder
    x0 = robot_state_coder.lcmcoder.jcoder.decode(msg.q0).val;
    q = x0(1:getNumDOF(r));
    try
      for i=1:msg.num_contact_goals
        goal = msg.contact_goals(i);
        if (goal.contact_type==goal.ON_GROUND_PLANE)
          body_ind=findLink(r,char(goal.object_1_name));
          collision_group = find(strcmpi(char(goal.object_1_contact_grp),body_ind.collision_group_name));
          if isempty(collision_group) error('couldn''t find collision group %s on body %s',char(goal.object_1_contact_grp),char(goal.object_1_name)); end
          p=[goal.target_pt.x; goal.target_pt.y; goal.target_pt.z];
          pos = struct();
          pos.max = inf(3,1);
          pos.min = -inf(3,1);
          if(goal.x_relation == 0)
              pos.min(1) = p(1)+goal.target_pt_radius;
              pos.max(1) = p(1)-goal.target_pt_radius;
          elseif(goal.x_relation == 1)
              pos.max(1) = p(1);
              pos.min(1) = -inf;
          elseif(goal.x_relation == 2)
              pos.min(1) = p(1);
              pos.max(1) = inf;
          elseif(goal.x_relation == 3)
              pos.min(1) = -inf;
              pos.max(1) = inf;
          end
          if(goal.y_relation == 0)
              pos.min(2) = p(2)-goal.target_pt_radius;
              pos.max(2) = p(2)+goal.target_pt_radius;
          elseif(goal.y_relation == 1)
              pos.max(2) = p(2);
              pos.min(2) = -inf;
          elseif(goal.y_relation == 2)
              pos.min(2) = p(2);
              pos.max(2) = inf;
          elseif(goal.y_relation == 3)
              pos.min(2) = -inf;
              pos.max(2) = inf;
          end
          if(goal.contact_type == 0)
              pos.min(3) = 0;
              pos.max(3) = 0;
          else
              error('Not implemented yet')
          end
%           if(goal.z_relation == 0)
%               pos.min(3) = max([0,p(3)-goal.target_pt_radius]);
%               pos.max(3) = max([pos.min(3),p(3)+goal.target_pt_radius]);
%           elseif(goal.z_relation == 1)
%               pos.max(3) = p(3);
%               pos.min(3) = 0;
%           elseif(goal.z_relation == 2)
%               pos.min(3) = max([0,p(2)]);
%               pos.max(3) = inf;
%           elseif(goal.z_relation == 3)
%               pos.min(3) = 0;
%               pos.max(3) = inf;
%           end
          tspan = [goal.lower_bound_completion_time goal.upper_bound_completion_time];
          collision_group_pt = mean(body_ind.getContactPoints(collision_group),2);
          action_constraint = ActionKinematicConstraint(r,body_ind,collision_group_pt,pos,tspan,[body_ind.linkname,'_from_',num2str(tspan(1)),'_to_',num2str(tspan(2))]);
          action_sequence = action_sequence.addKinematicConstraint(action_constraint);
        end
      end

      % Above ground constraints
      tspan = action_sequence.tspan;
      for body_ind = 1:length(r.body)
        body_contact_pts = r.body(body_ind).getContactPoints();
        if(~isempty(body_contact_pts))
          above_ground_constraint = ActionKinematicConstraint.groundConstraint(r,body_ind,body_contact_pts,tspan,[r.body(body_ind).linkname,'_above_ground_from_',num2str(tspan(1)),'_to_',num2str(tspan(2))]);
          action_sequence = action_sequence.addKinematicConstraint(above_ground_constraint);
        end 
      end

      % If the action sequence is specified, we need to solve the ZMP
      % planning and IK for the whole sequence.
      if(action_options.ZMP)
          num_key_time_samples = length(action_sequence.key_time_samples);
          com_key_time_samples = zeros(3,num_key_time_samples);
          q_key_time_samples = zeros(nq,num_key_time_samples);
          for i = 1:length(action_sequence.key_time_samples)
              % Solve the IK here to for each key time point (the starting of
              % ending of each constraint). We agree that the constraint is
              % active in the time interval [start_time,
              % end_time) (Half closed half open)
              ikargs = action_sequence.getIKArguments(action_sequence.key_time_samples(i));
              if isempty(ikargs)
                q_key_time_samples(:,i)=options.q_nom;
              else
                % call IK
                [q_key_time_samples(:,i),info] = inverseKin(r,q,ikargs{:},options);
                if(info>10)
                    warning(['IK at time ',num2str(action_sequence.key_time_samples(i)),' is not successful']);
                end
              end
              % if the q_start_time is in contact, then replace the inequality contact constraint with the equality
              % contact constraint
              kinsol = doKinematics(r,q_key_time_samples(:,i));
              com_key_time_samples(:,i) = getCOM(r,kinsol);
              j = 1;
              while(j<length(ikargs))
                  body_ind = ikargs{j};
                  if(~isnumeric(body_ind))
                      error('action sequence should only return numeric body_ind');
                  end
                  if(body_ind == 0)
                      body_pos_lb_time = ikargs{j+1};
                      action_constraint_ZMP = actionKinematicConstraint(r,body_ind,[0;0;0],body_pos_lb_time,[action_sequence.key_time_samples(i) action_sequence.key_time_samples(i)],['com_at_',num2str(action_sequence.key_time_samples(i))]);
                      j = j+2;
                  else
                      collision_group_pt = ikargs{j+1};
                      pos_start_time = forwardKin(r,kinsol,body_ind,collision_group_pt,false);
                      
                      contact_pos_ind = pos_start_time(3,:)<contact_tol;
                      if(~isempty(contact_pos_ind)&&i<length(action_sequence.key_time_samples))
                          tspan = action_sequence.key_time_samples(i:i+1);
                          pos_start_time_con = struct();
                          pos_start_time_con.max = pos_start_time(:,contact_pos_ind);
                          pos_start_time_con.min = pos_start_time(:,contact_pos_ind);
                          action_constraint_ZMP_grd_contact = ActionKinematicConstraint(r,body_ind,collision_group_pt(:,contact_pos_ind),pos_start_time_con,tspan,[r.body(body_ind).linkname,'_ground_contact_from_',num2str(tspan(1)),'_to_',num2str(tspan(2))]);
                          action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(action_constraint_ZMP_grd_contact);
                      end
                      
                      action_constraint_ZMP = ActionKinematicConstraint(r,body_ind,collision_group_pt,ikargs{j+2},[action_sequence.key_time_samples(i) action_sequence.key_time_samples(i)],[r.body(body_ind).linkname,'_at_',num2str(action_sequence.key_time_samples(i))]);
                      
                      j = j+3;
                  end
                  action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(action_constraint_ZMP);
              end
          end
        
        tspan = action_sequence.tspan;
        for body_ind = 1:length(r.body)
          body_contact_pts = r.body(body_ind).getContactPoints();
          if(~isempty(body_contact_pts))
            above_ground_constraint = ActionKinematicConstraint.groundConstraint(r,body_ind,body_contact_pts,tspan,[r.body(body_ind).linkname,'_above_ground_from_',num2str(tspan(1)),'_to_',num2str(tspan(2))]);
            action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(above_ground_constraint);
          end 
        end
        
        dt = 0.1;
          window_size = ceil((action_sequence_ZMP.tspan(end)-action_sequence_ZMP.tspan(1))/dt);
          zmp_planner = ZMPplanner(window_size,r.num_contacts,dt,9.81);
          t_breaks = action_sequence_ZMP.tspan(1)+dt*(0:window_size-1);
          t_breaks(end) = action_sequence_ZMP.tspan(end);
          contact_pos = cell(1,window_size);
          for i = 1:length(t_breaks)
              ikargs = action_sequence_ZMP.getIKArguments(t_breaks(i));
              j = 1;
              while j<length(ikargs)
                  if(ikargs{j} == 0)
                    j = j+2;
                  else
                      contact_pos_ind = (ikargs{j+2}.max(3,:)<contact_tol)&(all(ikargs{j+2}.max(1:2,:)==ikargs{j+2}.min(1:2,:),1));
                      contact_pos{i} = [contact_pos{i} ikargs{j+2}.max(1:2,contact_pos_ind)];
                      j = j+3;
                  end
              end
          end
%           contact_pos = zeros(2,r.num_contacts, window_size);
%           contact_flag = false(r.num_contacts,window_size);
%           for i = 1:length(t_breaks)
%               ikargs = action_sequence_ZMP.getIKArguments(t_breaks(i));
%               num_contacts = 0;
%               for j = 3:3:length(ikargs)
%                   for k = 1:size(ikargs{j}.max,2)
%                       if(ikargs{j}.max(3,k)<contact_tol)
%                           num_contacts = num_contacts+1;
%                           contact_pos(:,num_contacts,i) = mean([ikargs{j}.max(1:2,k) ikargs{j}.min(1:2,k)],2);
%                           contact_flag(num_contacts,i) = true;
%                       end
%                   end
%               end
%           end
          % solve the IK for the key time samples, then interpolate the COM
          % height as the desired COM height
%           q_key_time_samples = zeros(r.getNumDOF(),length(action_sequence.key_time_samples));
%           com_key_time_samples = zeros(3,length(action_sequence.key_time_samples));
%           for i = 1:length(action_sequence.key_time_samples)
%               ikargs = action_sequence.getIKArguments(action_sequence.key_time_samples(i));
%               if(isempty(ikargs))
%                   q_key_time_samples(:,i) = options.q_nom;
%               else
%                   q_key_time_samples(:,i) = inverseKin(r,q,ikargs{:},options);
%               end
%               com_key_time_samples(:,i) = r.getCOM(q_key_time_samples(:,i));
%           end
          % TODO: Publish constraint satisfaction message here
          com_height_traj = PPTrajectory(foh(action_sequence.key_time_samples,com_key_time_samples(3,:)));
          com_height = com_height_traj.eval(t_breaks);
          q0 = q;
          com0 = r.getCOM(q0);
          comdot0 = 0*com0;
          zmp_options = struct();
          zmp_options.supportPolygonConstraints = false;
          zmp_options.shrink_factor = 0.8;
          zmp_options.useQP = true;
          zmp_options.penalizeZMP = true;
          [com_plan,planar_comdot_plan,~,zmp_plan] = zmp_planner.planning(com0(1:2),comdot0(1:2),contact_pos,com_height,t_breaks,zmp_options);
%           q_zmp_plan = zeros(r.getNumDOF,length(t_breaks));
%           q_zmp_plan(:,1) = q0;

          % Add com constraints to action_sequence
          comdot_height_plan = com_height_traj.deriv(t_breaks);
          comdot_plan = [planar_comdot_plan;comdot_height_plan];
          com_traj = PPTrajectory(pchipDeriv(t_breaks,com_plan,comdot_plan));
          
          com_constraint = ActionKinematicConstraint(r,0,zeros(3,1),com_traj, ...
                              action_sequence_ZMP.tspan,'com');
          action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(com_constraint);
          
          options.qtraj0 = PPTrajectory(spline(action_sequence.key_time_samples,q_key_time_samples));
          options.quasiStaticFlag = false;
          options.nSample = length(t_breaks)-1;
          [q_zmp_traj,inverse_kin_sequence_info] = inverseKinSequence(r, q0, action_sequence,options);
          t_zmp_breaks = q_zmp_traj.getBreaks();
          q_zmp_plan = q_zmp_traj.eval(t_zmp_breaks);
          qdot_zmp_plan = q_zmp_traj.deriv(t_zmp_breaks);
          xtraj = PPTrajectory(pchipDeriv(t_zmp_breaks,[q_zmp_plan;qdot_zmp_plan],[qdot_zmp_plan;0*qdot_zmp_plan]));
          xtraj = xtraj.setOutputFrame(r.getStateFrame());
          v.playback(xtraj,struct('slider',true));
          % publish t_breaks, q_zmp_plan with RobotPlanPublisher.java
          publish(robot_plan_publisher, q_zmp_traj.getBreaks(), ...
                  [q_zmp_plan; zeros(size(q_zmp_plan))]);
      end
      ikargs = action_sequence.getIKArguments(action_sequence.tspan(end));
      if isempty(ikargs)
        q=options.q_nom;
      else
        % call IK
        q = inverseKin(r,q,ikargs{:},options);
      end
    catch ex
      warning(ex.identifier,ex.message);
      q=q_bk;
      continue;
    end
    % publish robot state message
    x = [q;0*q];
    v.draw(0,x);
    publish(robot_state_coder,0,x,'ACTION_AUTHORING_IK_ROBOT_STATE');
  end
end

end 
