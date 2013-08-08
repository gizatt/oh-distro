classdef QPControlBlock < MIMODrakeSystem

  methods
  function obj = QPControlBlock(r,controller_data,options)
    % @param r atlas instance
    % @param controller_data shared data handle containing linear system, zmp trajectories, Riccati solution, etc
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and input cost (R)
    typecheck(r,'Atlas');
    typecheck(controller_data,'SharedDataHandle');

    QPControlBlock.check_ctrl_data(controller_data)
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r); % input frame for desired qddot 
    ft_frame = AtlasForceTorque();

    if isfield(options,'multi_robot')
      typecheck(options.multi_robot,'TimeSteppingRigidBodyManipulator');
      fr = options.multi_robot.getStateFrame;
      % IMPORTANT NOTE: I'm assuming the atlas state is always the first
      % frame in a multi coordinate frame
      if typecheck(fr,'MultiCoordinateFrame')
        input_frame = MultiCoordinateFrame({qddframe,ft_frame,fr.frame{:}});
        num_state_fr = length(options.multi_robot.getStateFrame.frame);
      else
        input_frame = MultiCoordinateFrame({qddframe,ft_frame,fr});
        num_state_fr = 1;
      end
    else
      input_frame = MultiCoordinateFrame({qddframe,ft_frame,r.getStateFrame});
      num_state_fr = 1;
    end
    
    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.004;
    end
    
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.controller_data = controller_data;
    obj.num_state_frames = num_state_fr;
    
    if isfield(options,'multi_robot')
      obj.multi_robot = options.multi_robot;
    else
      obj.multi_robot = 0;
    end
   
    % weight for desired qddot objective term
    if isfield(options,'w')
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    else
      obj.w = 0.1;
    end
    
    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end
    
    if isfield(options,'R')
      warning('input cost no longer supported');
    end
    
    if ~isfield(options,'lcm_foot_contacts')
      obj.lcm_foot_contacts=true; % listen for foot contacts over LCM
    else
      typecheck(options.lcm_foot_contacts,'logical');
      obj.lcm_foot_contacts=options.lcm_foot_contacts;
    end
    
    if isfield(options,'debug')
      typecheck(options.debug,'logical');
      sizecheck(options.debug,1);
      obj.debug = options.debug;
    else
      obj.debug = false;
    end

    if isfield(options,'use_mex')
      % 0 - no mex
      % 1 - use mex
      % 2 - run mex and non-mex and valuecheck the result
      sizecheck(options.use_mex,1);
      obj.use_mex = uint32(options.use_mex);
      rangecheck(obj.use_mex,0,2);
      if (obj.use_mex && exist('QPControllermex')~=3)
        error('can''t find QPControllermex.  did you build it?');
      end
    else
      obj.use_mex = 1;
    end

    if isfield(options,'use_hand_ft')
      obj.use_hand_ft = options.use_hand_ft;
    else
      obj.use_hand_ft = false;
    end

    if isfield(options,'include_angular_momentum')
      obj.include_angular_momentum = options.include_angular_momentum;
    else
      obj.include_angular_momentum = false;
    end

    if (isfield(options,'ignore_states'))
      % specifies what dimensions of the state we should ignore 
      assert(isnumeric(options.ignore_states));
      rangecheck(options.ignore_states,1,getNumStates(r));
      assert(isfield(controller_data.data,'xtraj'));
      assert(isa(controller_data.data.xtraj,'Trajectory'));
      obj.ignore_states = options.ignore_states;
    else
      obj.ignore_states = [];
    end
    
    % specifies whether or not to solve QP for all DOFs or just the
    % important subset
    if (isfield(options,'full_body_opt'))
      warning('full_body_opt option no longer supported --- controller is always full body.')
    end

    obj.lc = lcm.lcm.LCM.getSingleton();
    obj.rfoot_idx = findLinkInd(r,'r_foot');
    obj.lfoot_idx = findLinkInd(r,'l_foot');
    obj.rhand_idx = findLinkInd(r,'r_hand');
    obj.lhand_idx = findLinkInd(r,'l_hand');
    obj.nq = getNumDOF(r);
    
    if obj.lcm_foot_contacts
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
    end % else estimate contact via kinematics
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% NOTE: these parameters need to be set in QPControllermex.cpp, too %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    obj.solver_options.outputflag = 0; % not verbose
    obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
    obj.solver_options.presolve = 0;
    % obj.solver_options.prepasses = 1;

    if obj.solver_options.method == 2
      obj.solver_options.bariterlimit = 20; % iteration limit
      obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
      obj.solver_options.barconvtol = 5e-4;
    end
    
    if (obj.use_mex>0)
      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      if isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
        multi_robot_ptr = obj.multi_robot.getMexModelPtr.ptr;
      else
        multi_robot_ptr = 0;
      end
      obj.mex_ptr = SharedDataHandle(QPControllermex(0,obj,obj.robot.getMexModelPtr.ptr,getB(obj.robot),r.umin,r.umax,terrain_map_ptr,multi_robot_ptr));
    end

    obj.num_body_contacts=zeros(getNumBodies(r),1);
    for i=1:getNumBodies(r)
      obj.num_body_contacts(i) = length(getBodyContacts(r,i));
    end
    
    
    if isa(getTerrain(r),'DRCFlatTerrainMap')
      obj.using_flat_terrain = true;      
    else
      obj.using_flat_terrain = false;
    end
    
    obj.lcmgl = bot_lcmgl_init('qp-control-block-debug');

    [obj.jlmin, obj.jlmax] = getJointLimits(r);
        
  end

  end
  
  methods (Static)
    function check_ctrl_data(ctrl_data)
      if ~isfield(ctrl_data.data,'D')
        % assumed  ZMP system
        hddot = 0; % could use estimated comddot here
        ctrl_data.setField('D',-0.89/(hddot+9.81)*eye(2)); % TMP hard coding height here. Could be replaced with htraj from planner
        % or current height above height map;
      end
      if ~isfield(ctrl_data.data,'qp_active_set')
        ctrl_data.setField('qp_active_set',[]);
      end
      
      ctrl_data = ctrl_data.data;
      
      % i've made the following assumptions to make things fast.  we can soften
      % them later as desired.  - Russ
      assert(isnumeric(ctrl_data.Qy));
%       sizecheck(ctrl_data.Qy,[2 2]); % commented out by sk--some of the
%       quasistatic systems pass 4x4 Qs
      assert(isnumeric(ctrl_data.R));
      sizecheck(ctrl_data.R,[2 2]);
      assert(isnumeric(ctrl_data.C));
      
      assert(isnumeric(ctrl_data.S));
      sizecheck(ctrl_data.S,[4 4]);
      assert(isnumeric(ctrl_data.x0));
      sizecheck(ctrl_data.x0,[4 1]);
      assert(isnumeric(ctrl_data.u0));
      if ctrl_data.is_time_varying
        assert(isa(ctrl_data.s1,'Trajectory'));
        assert(isa(ctrl_data.s2,'Trajectory'));
        assert(isa(ctrl_data.s1dot,'Trajectory'));
        assert(isa(ctrl_data.s2dot,'Trajectory'));
        assert(isa(ctrl_data.y0,'Trajectory'));
      else
        assert(isnumeric(ctrl_data.s1));
        assert(isnumeric(ctrl_data.s2));
        assert(isnumeric(ctrl_data.y0));
%        sizecheck(ctrl_data.supports,1);  % this gets initialized to zero
%        in constructors.. but doesn't get used.  would be better to
%        enforce it.
      end       
      sizecheck(ctrl_data.s1,[4 1]);
      sizecheck(ctrl_data.s2,1);
      assert(isnumeric(ctrl_data.mu));
      assert(islogical(ctrl_data.ignore_terrain));
    end
  end
  
  methods
    
  function y=mimoOutput(obj,t,~,varargin)
%    out_tic = tic;
    ctrl_data = obj.controller_data.data;
    
%    QPControlBlock.check_ctrl_data(ctrl_data);  % todo: remove this after all of the DRC Controllers call it reliably on their initialize method
  
    q_ddot_des = varargin{1};
    ft = varargin{2};
    hand_ft = ft(6+(1:12));
    % IMPORTANT NOTE: I'm assuming the atlas state is always the first
    % frame in a multi coordinate frame 
    x = varargin{3};
    
    if ~isempty(obj.ignore_states)
      xt=fasteval(ctrl_data.xtraj,t);
      x(obj.ignore_states) = xt(obj.ignore_states);
    end
    
    r = obj.robot;
    nq = obj.nq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

    q_multi = q;
    for i=1:obj.num_state_frames-1
      xi = varargin{3+i};
      q_multi = [q_multi; xi(1:end/2)];
    end
            
    %----------------------------------------------------------------------
    % Linear system stuff for zmp/com control -----------------------------
    A_ls = ctrl_data.A; % always TI
    B_ls = ctrl_data.B; % always TI
    Qy = ctrl_data.Qy;
    R_ls = ctrl_data.R;
    C_ls = ctrl_data.C;
    D_ls = ctrl_data.D;
    S = ctrl_data.S;
    Sdot = 0*S; % constant for ZMP/double integrator dynamics
    x0 = ctrl_data.x0 - [ctrl_data.trans_drift(1:2);0;0]; % for x-y plan adjustment
    u0 = ctrl_data.u0;
    if (ctrl_data.is_time_varying)
      s1 = fasteval(ctrl_data.s1,t);
%       s2 = fasteval(ctrl_data.s2,t);
      s1dot = fasteval(ctrl_data.s1dot,t);
      s2dot = fasteval(ctrl_data.s2dot,t);
      y0 = fasteval(ctrl_data.y0,t) - ctrl_data.trans_drift(1:2); % for x-y plan adjustment
      
      %----------------------------------------------------------------------
      % extract current supports
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
    else
      s1 = ctrl_data.s1;
%       s2 = ctrl_data.s2;
      s1dot = 0*s1;
      s2dot = 0;
      y0 = ctrl_data.y0 - ctrl_data.trans_drift(1:2); % for x-y plan adjustment
      
      supp = ctrl_data.supports;
    end
    mu = ctrl_data.mu;
    R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;

    % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
    contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all
    if obj.lcm_foot_contacts
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
      if ~isempty(contact_data)
        msg = drc.foot_contact_estimate_t(contact_data);
        contact_sensor(find(supp.bodies==obj.lfoot_idx)) = msg.left_contact;
        contact_sensor(find(supp.bodies==obj.rfoot_idx)) = msg.right_contact;
      end
    end
    
    contact_threshold = 0.001; % a point is considered to be in contact if within this distance
    if (obj.use_mex==0 || obj.use_mex==2)

      % Change in logic here due to recent tests with heightmap noise
      % for now, we will do a logical OR of the force-based sensor and the
      % kinematic criterion for foot contacts
      %
      % another option would be to limit forces on the feet when kinematics
      % says 'contact', but force sensors do not. when both agree, allow full
      % forces on the feet
      kinsol = doKinematics(r,q,false,true);

      if any(supp.contact_surfaces~=0) && isa(obj.multi_robot,'TimeSteppingRigidBodyManipulator')
        kinsol_multi = doKinematics(obj.multi_robot,q_multi,false,true); % for now assume the same state frame
      end
      
      % get active contacts
      i=1;
      while i<=length(supp.bodies)
        if ctrl_data.ignore_terrain
          % use all desired supports UNLESS we have sensor information saying no contact
          if (contact_sensor(i)==0) 
            supp = removeBody(supp,i); 
            contact_sensor(i)=[];
            i=i-1;
          end
        else
          % check kinematic contact
          if supp.contact_surfaces(i) == 0
            phi = contactConstraints(r,kinsol,supp.bodies(i),supp.contact_pts{i});
          else
            % use bullet collision between bodies
            phi = pairwiseContactConstraints(obj.multi_robot,kinsol_multi,supp.bodies(i),supp.contact_surfaces(i),supp.contact_pts{i});
          end
          contact_state_kin = any(phi<=contact_threshold);
          
          if (~contact_state_kin && contact_sensor(i)<1) 
            % no contact from kin, no contact (or no info) from sensor
            supp = removeBody(supp,i); 
            contact_sensor(i)=[];
            i=i-1;
          end
        end
        i=i+1;
      end
      active_supports = (supp.bodies)';
      active_surfaces = supp.contact_surfaces;
      active_contact_pts = supp.contact_pts;
      num_active_contacts = supp.num_contact_pts;      
        
      %----------------------------------------------------------------------
      % Disable hand force/torque contribution to dynamics as necessary
      if (~obj.use_hand_ft)
        hand_ft=0*hand_ft;
      else
        if any(active_supports==obj.lhand_idx)
          hand_ft(1:6)=0;
        end
        if any(active_supports==obj.rhand_idx)
          hand_ft(7:12)=0;
        end
      end
          
      %----------------------------------------------------------------------
      
      nu = getNumInputs(r);
      nq = getNumDOF(r);
      dim = 3; % 3D
      nd = 4; % for friction cone approx, hard coded for now
      float_idx = 1:6; % indices for floating base dofs
      act_idx = 7:nq; % indices for actuated dofs

      kinsol = doKinematics(r,q,false,true,qd);
      
      [H,C,B] = manipulatorDynamics(r,q,qd);

      [~,Jlhand] = forwardKin(r,kinsol,obj.lhand_idx,zeros(3,1),1);
      [~,Jrhand] = forwardKin(r,kinsol,obj.rhand_idx,zeros(3,1),1);
      C = C + Jlhand'*hand_ft(1:6) + Jrhand'*hand_ft(7:12);
      
      H_float = H(float_idx,:);
      C_float = C(float_idx);

      H_act = H(act_idx,:);
      C_act = C(act_idx);
      B_act = B(act_idx,:);

      [xcom,J] = getCOM(r,kinsol);
      
      if obj.include_angular_momentum
        [Ag,Agdot] = getCMM(r,kinsol,qd);
  
        Ag_ang = Ag(1:3,:);
        Agdot_ang = Agdot(1:3,:);
        h_ang_dot_des = [0;0;0]; % regulate to zero for now
        w2 = 0.0001; % QP objective function weight
        
        if 0
          % plot momentum vectors
          h = Ag*qd;
          bot_lcmgl_line_width(obj.lcmgl, 3);

          bot_lcmgl_push_matrix(obj.lcmgl);
          bot_lcmgl_translated(obj.lcmgl,xcom(1),xcom(2),xcom(3));
          bot_lcmgl_color3f(obj.lcmgl, 1, 0, 0);
          bot_lcmgl_begin(obj.lcmgl, obj.lcmgl.LCMGL_LINES);
          bot_lcmgl_vertex3f(obj.lcmgl, 0, 0, 0);
          bot_lcmgl_vertex3f(obj.lcmgl, 0.05*h(4), 0.05*h(5), 0.05*h(6)); % scale for drawing
          bot_lcmgl_end(obj.lcmgl);

          aa_h = rpy2axis(h(1:3));
          bot_lcmgl_color3f(obj.lcmgl, 0, 1, 0);
          bot_lcmgl_begin(obj.lcmgl, obj.lcmgl.LCMGL_LINES);
          bot_lcmgl_vertex3f(obj.lcmgl, 0, 0, 0);
          bot_lcmgl_vertex3f(obj.lcmgl, 0.2*aa_h(1), 0.2*aa_h(2), 0.2*aa_h(3)); % scale for drawing
          bot_lcmgl_end(obj.lcmgl);
          bot_lcmgl_pop_matrix(obj.lcmgl);

          bot_lcmgl_line_width(obj.lcmgl, 1);
          bot_lcmgl_switch_buffer(obj.lcmgl);
        end
      end
      
      Jdot = forwardJacDot(r,kinsol,0);
      J = J(1:2,:); % only need COM x-y
      Jdot = Jdot(1:2,:);
      
      if ~isempty(active_supports)
        nc = sum(num_active_contacts);
        c_pre = 0;
        Dbar = [];
        for j=1:length(active_supports)
          if active_surfaces(j) == 0
            [~,B,JB] = contactConstraintsBV(r,kinsol,active_supports(j),active_contact_pts{j});
            if 0
              cpos = contactPositions(r,kinsol,active_supports(j),active_contact_pts{j});
              acpts = active_contact_pts{j};
              for jj=1:length(acpts)
                b=0.1*B{jj};
                m = size(b,2); 
                bot_lcmgl_line_width(obj.lcmgl, 3);
                bot_lcmgl_push_matrix(obj.lcmgl);
                bot_lcmgl_translated(obj.lcmgl,cpos(1,jj),cpos(2,jj),cpos(3,jj));
                bot_lcmgl_color3f(obj.lcmgl, 1, 0, 0);
                for kk=1:m
                  bot_lcmgl_begin(obj.lcmgl, obj.lcmgl.LCMGL_LINES);
                  bot_lcmgl_vertex3f(obj.lcmgl, 0, 0, 0);
                  bot_lcmgl_vertex3f(obj.lcmgl, b(1,kk), b(2,kk), b(3,kk)); 
                  bot_lcmgl_end(obj.lcmgl);
                end
                bot_lcmgl_pop_matrix(obj.lcmgl);
              end
            end
          else
            % use bullet collision between bodies
            [~,~,JB] = pairwiseContactConstraintsBV(obj.multi_robot,kinsol_multi,active_supports(j),active_surfaces(j),active_contact_pts{j});
          end
          Dbar = [Dbar, [JB{:}]];
          c_pre = c_pre + length(active_contact_pts{j});

        end
        bot_lcmgl_line_width(obj.lcmgl, 1);
        bot_lcmgl_switch_buffer(obj.lcmgl);

        Dbar_float = Dbar(float_idx,:);
        Dbar_act = Dbar(act_idx,:);

        [cpos,Jp,Jpdot] = contactPositionsJdot(r,kinsol,active_supports,active_contact_pts);
        Jp = sparse(Jp);
        Jpdot = sparse(Jpdot);
        
        xlimp = [xcom(1:2); J*qd]; % state of LIP model
        x_bar = xlimp - x0;      
      else
        nc = 0;
      end
      neps = nc*dim;

         
      %----------------------------------------------------------------------
      % Build handy index matrices ------------------------------------------
      
      nf = nc*nd; % number of contact force variables
      nparams = nq+nf+neps;
      Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
      Ibeta = zeros(nf,nparams); Ibeta(:,nq+(1:nf)) = eye(nf);
      Ieps = zeros(neps,nparams);
      Ieps(:,nq+nf+(1:neps)) = eye(neps);
      
      
      %----------------------------------------------------------------------
      % Set up problem constraints ------------------------------------------
      
      lb = [-1e3*ones(1,nq) zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
      ub = [ 1e3*ones(1,nq) 500*ones(1,nf) obj.slack_limit*ones(1,neps)]';
      
      % if at joint limit, disallow accelerations in that direction
      lb(q<=obj.jlmin+1e-4) = 0;
      ub(q>=obj.jlmax-1e-4) = 0;
      
      Aeq_ = cell(1,2);
      beq_ = cell(1,2);
      Ain_ = cell(1,2);
      bin_ = cell(1,2);
      
      % constrained dynamics
      if nc>0
        Aeq_{1} = H_float*Iqdd - Dbar_float*Ibeta;
      else
        Aeq_{1} = H_float*Iqdd;
      end
      beq_{1} = -C_float;

      % input saturation constraints
      % u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)

      if nc>0
        Ain_{1} = B_act'*(H_act*Iqdd - Dbar_act*Ibeta);
      else
        Ain_{1} = B_act'*H_act*Iqdd;
      end
      bin_{1} = -B_act'*C_act + r.umax;
      Ain_{2} = -Ain_{1};
      bin_{2} = B_act'*C_act - r.umin;

      if nc > 0
        % relative acceleration constraint
        Aeq_{2} = Jp*Iqdd + Ieps;
        beq_{2} = -Jpdot*qd - 1.0*Jp*qd;
      end
      
      % linear equality constraints: Aeq*alpha = beq
      Aeq = sparse(vertcat(Aeq_{:}));
      beq = vertcat(beq_{:});
      
      % linear inequality constraints: Ain*alpha <= bin
      Ain = sparse(vertcat(Ain_{:}));
      bin = vertcat(bin_{:});

    
      %----------------------------------------------------------------------
      % QP cost function ----------------------------------------------------
      %
      %  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x_bar+D*(Jdot*qd + J*qdd),Qy) + (2*x_bar'*S + s1')*(A*x_bar + B*(Jdot*qd + J*qdd-u0)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
      
      if nc > 0
        Hqp = Iqdd'*J'*R_DQyD_ls*J*Iqdd;
        Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + obj.w*eye(nq);
        if obj.include_angular_momentum
          Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + w2*Ag_ang'*Ag_ang;
        end
        
        fqp = xlimp'*C_ls'*Qy*D_ls*J*Iqdd;
        fqp = fqp + qd'*Jdot'*R_DQyD_ls*J*Iqdd;
        fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*J*Iqdd;
        fqp = fqp - u0'*R_ls*J*Iqdd;
        fqp = fqp - y0'*Qy*D_ls*J*Iqdd;
        fqp = fqp - obj.w*q_ddot_des'*Iqdd;
        if obj.include_angular_momentum
          fqp = fqp + w2*qd'*Agdot_ang'*Ag_ang*Iqdd;
          fqp = fqp - w2*h_ang_dot_des'*Ag_ang*Iqdd;
        end
        
        % quadratic slack var cost 
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = 0.001*eye(neps); 
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -q_ddot_des'*Iqdd;
      end

      %----------------------------------------------------------------------
      % Solve QP ------------------------------------------------------------
      
      REG = 1e-8;
      
      IR = eye(nparams);  
      lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
      Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
      bin_fqp = [bin; -lb(lbind); ub(ubind)];

      if obj.use_mex ~= 2
        % call fastQPmex first
        QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq),zeros(nf,1)+ REG*ones(nf,1),0.001*ones(neps,1)+ REG*ones(neps,1)};
        Aeq_fqp = full(Aeq);

        %% NOTE: model.obj is 2* f for fastQP!!!
        [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Aeq_fqp,beq,Ain_fqp,bin_fqp,ctrl_data.qp_active_set);
      else
        info_fqp = -1;
      end
      
      if info_fqp<0
        % then call gurobi
        
        model.Q = sparse(Hqp + REG*eye(nparams));
        model.A = [Aeq; Ain];
        model.rhs = [beq; bin];
        model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
        model.lb = lb;
        model.ub = ub;

        model.obj = fqp;
        if ~isempty(model.A) && obj.solver_options.method==2
          % see drake/algorithms/test/mygurobi.m
          model.obj = 2*model.obj;
        end

        if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
          keyboard;
        end

%         qp_tic = tic;
        result = gurobi(model,obj.solver_options);
%         qp_toc = toc(qp_tic);
%         fprintf('QP solve: %2.4f\n',qp_toc);
      
        alpha = result.x;

        if isempty(model.A) && obj.solver_options.method==2
          % see drake/algorithms/test/mygurobi.m
          alpha = alpha/2;
        end
      end
        
      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      setField(obj.controller_data,'qp_active_set',qp_active_set);
      
      %----------------------------------------------------------------------
      % Solve for inputs ----------------------------------------------------

      qdd = alpha(1:nq);
      if nc>0
        beta = alpha(nq+(1:nf));
        u = B_act'*(H_act*qdd + C_act - Dbar_act*beta);
      else
        u = B_act'*(H_act*qdd + C_act);
      end
      y = u;
 
      if (obj.use_mex==2)
        des.y = y;
      end
      
      % compute V,Vdot for controller status updates
      if (nc>0)
        %V = x_bar'*S*x_bar + s1'*x_bar + s2;
        %Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*Sdot*x_bar + x_bar'*s1dot + s2dot;
        % note for ZMP dynamics, S is constant so Sdot=0
      
        Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*s1dot + s2dot;
      end
    end
  
    if (obj.use_mex==1)
      if ctrl_data.ignore_terrain
        contact_threshold =-1;       
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,Vdot,active_supports] = QPControllermex(obj.mex_ptr.data,1,q_ddot_des,x,q_multi, ...
          supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,s1dot,s2dot,x0,u0,y0,mu, ...
          contact_sensor,contact_threshold,height,obj.include_angular_momentum);
    end

    if ~isempty(active_supports)
      setVdot(obj.controller_data,Vdot);
    else
      setVdot(obj.controller_data,0);
    end
    
    if (obj.use_mex==2)
      % note: this only works when using gurobi
      if ctrl_data.ignore_terrain
        contact_threshold =-1;       
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      [y,Vdotmex,active_supports_mex,Q,gobj,A,rhs,sense,lb,ub] = QPControllermex(obj.mex_ptr.data, ...
        0,q_ddot_des,x,q_multi,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,s1dot,s2dot, ...
        x0,u0,y0,mu,contact_sensor,contact_threshold,height,obj.include_angular_momentum);
      if (nc>0)
        valuecheck(active_supports_mex,active_supports);
        valuecheck(Vdotmex,Vdot,1e-3);
      end
      valuecheck(Q'+Q,model.Q'+model.Q,1e-12);
      valuecheck(gobj,model.obj,1e-12);
      valuecheck(A,model.A,1e-12);
      valuecheck(rhs,model.rhs,1e-12);
      valuecheck(sense',model.sense);
      valuecheck(lb,model.lb,1e-12);
      valuecheck(ub,model.ub,1e-12);
%       valuecheck(y,des.y,0.5);
    end
    
   
    if obj.debug && (obj.use_mex==0 || obj.use_mex==2) && nc > 0
%       if nq_free > 0
%         xcomdd = Jdot * qd + J * qdd;
%       else
%         xcomdd = Jdot * qd + J * alpha(1:nq);
%       end
%       zmppos = xcom(1:2) + D_ls * xcomdd;
%       convh = convhull(cpos(1,:), cpos(2,:));
%       zmp_ok = inpolygon(zmppos(1), zmppos(2), cpos(1,convh), cpos(2,convh));
%       if zmp_ok
%         color = [0 1 0];
%       else
%         color = [1 0 0];
%       end
%       plot_lcm_points([zmppos', mean(cpos(3,:))], color, 660, 'Commanded ZMP', 1, true);

      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      lax = find(~cellfun(@isempty,strfind(state_names,'l_leg_lax')));
      uay = find(~cellfun(@isempty,strfind(state_names,'l_leg_uay')));

      scope('Atlas','lax_qdd_des',t,q_ddot_des(lax),struct('linespec','r','scope_id',1));
      scope('Atlas','uay_qdd_des',t,q_ddot_des(uay),struct('linespec','b','scope_id',1));
      scope('Atlas','lax_qdd',t,qdd(lax),struct('linespec','r','scope_id',2));
      scope('Atlas','uay_qdd',t,qdd(uay),struct('linespec','b','scope_id',2));
      
      [~,normals] = getTerrainHeight(r,cpos);
      d = RigidBodyManipulator.surfaceTangents(normals);

      lambda = Iz*alpha;
      beta_full = Ibeta*alpha;
      for kk=1:8
        if kk<=nc
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.25*normals(:,kk)]', [0 0 1; 0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          beta = beta_full((kk-1)*nd+(1:nd),:);
          fvec = lambda(kk)*normals(:,kk) + d{1}(:,kk)*beta(1) + d{2}(:,kk)*beta(2) - d{1}(:,kk)*beta(3) - d{2}(:,kk)*beta(4);
          plot_lcm_points([cpos(:,kk) cpos(:,kk)+0.0025*fvec]', [1 0 0; 1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        else
          plot_lcm_points(zeros(2,3), [0 0 1;0 0 1], 23489083+kk, sprintf('Foot Contact Normal %d',kk), 2, true);
          plot_lcm_points(zeros(2,3), [1 0 0;1 0 0], 6643+kk, sprintf('Foot Contact Force %d',kk), 2, true);
        end
      end
    end

    if (0)     % simple timekeeping for performance optimization
      % note: also need to uncomment tic at very top of this method
      out_toc=toc(out_tic);
      persistent average_tictoc average_tictoc_n;
      if isempty(average_tictoc)
        average_tictoc = out_toc;
        average_tictoc_n = 1;
      else
        average_tictoc = (average_tictoc_n*average_tictoc + out_toc)/(average_tictoc_n+1);
        average_tictoc_n = average_tictoc_n+1;
      end
      if mod(average_tictoc_n,50)==0
        fprintf('Average control output duration: %2.4f\n',average_tictoc);
      end
    end
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    nq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w; % objective function weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    rfoot_idx;
    lfoot_idx;
    rhand_idx;
    lhand_idx;
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
    debug;
    use_mex;
    use_hand_ft;
    mex_ptr;
    lc;
    contact_est_monitor;
    lcm_foot_contacts;  
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    num_body_contacts; % vector of num contacts for each body
    multi_robot;
    num_state_frames; % if there's a multi robot defined this is 1+ the number of other state frames
    using_flat_terrain; % true if using DRCFlatTerrain
    ignore_states; % array if state indices we want to ignore (and substitute with planned values)
    lcmgl;
    include_angular_momentum; % tmp flag for testing out angular momentum control
    jlmin;
    jlmax;
    end
end
