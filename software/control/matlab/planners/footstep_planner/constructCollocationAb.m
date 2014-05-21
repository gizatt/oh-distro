function [A, b, Aeq, beq, step_map] = constructCollocationAb(biped, seed_plan, params)

  nsteps = length(seed_plan.footsteps) - 1;
  nv = 12 * nsteps;

  step_map.ineq = containers.Map('KeyType', 'int32', 'ValueType', 'any');
  step_map.eq = containers.Map('KeyType', 'int32', 'ValueType', 'any');

  A = [];
  b = [];
  offset = 0;
  for j = 2:nsteps
    [A_reach, b_reach] = biped.getReachabilityPolytope(seed_plan.footsteps(j).body_idx, seed_plan.footsteps(j+1).body_idx, params);
    A = [A; zeros(length(b_reach), nv)];
    b = [b; b_reach];
    con_ndx = offset + (1:length(b_reach));
    var_ndx = (j-1)*12+7:j*12;
    A(con_ndx, var_ndx) = A_reach;
    step_map.ineq(j) = con_ndx;
    offset = offset + length(b_reach);
  end

  Aeq = zeros(4*(nsteps),nv);
  beq = zeros(4*(nsteps),1);
  con_ndx = 1:4;
  x1_ndx = 1:6;
  dx_ndx = 7:12;
  Aeq(con_ndx, x1_ndx([1,2,3,6])) = -diag(ones(4,1));
  Aeq(con_ndx, dx_ndx([1,2,3,6])) = diag(ones(4,1));
  for j = 2:nsteps
    con_ndx = (j-1)*4+(1:4);
    x1_ndx = (j-2)*12+(1:6);
    x2_ndx = (j-1)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    Aeq(con_ndx, x1_ndx(3:6)) = -diag(ones(4,1));
    Aeq(con_ndx, x2_ndx(3:6)) = diag(ones(4,1));
    Aeq(con_ndx, dx_ndx(3:6)) = -diag(ones(4,1));
    step_map.eq(j) = con_ndx;
  end
end