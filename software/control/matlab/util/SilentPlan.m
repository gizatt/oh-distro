classdef SilentPlan < QPControllerPlanMatlabImplementation  
  properties
    spline
  end

  methods
    function obj = SilentPlan(robot)
      obj.spline = drake.lcmt_piecewise_polynomial;
      obj.spline.num_breaks = 0;
      obj.spline.num_segments = 0;
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = drake.lcmt_qp_controller_input;
      qp_input.zmp_data = drake.lcmt_zmp_data;
      qp_input.whole_body_data = drake.lcmt_whole_body_data;
      qp_input.whole_body_data.spline = obj.spline;
      qp_input.whole_body_data.num_constrained_dofs = 0;
      qp_input.param_set_name = '';
      qp_input.be_silent = true;
    end
  end
end
