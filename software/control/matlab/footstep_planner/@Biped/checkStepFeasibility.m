function c = checkStepFeasibility(biped, p0, pf, p0_is_right_foot)

  sizecheck(p0(1,:), size(p0_is_right_foot));

  if size(p0, 1) == 3
    p0 = [p0(1, :); p0(2, :); zeros(3, size(p0, 2)); p0(3, :)];
  end


  max_forward_step = 0.35;
  max_backward_step = 0.15;
  max_step_width = 0.35;
  min_step_width = 0.20;

  y_max = max_step_width;
  y_min = min_step_width;
  y_mean = mean([y_max, y_min]);

  x_max = max_forward_step;
  x_min = -max_backward_step;
  x_mean = mean([x_max, x_min]);

  c = zeros(3, size(p0, 2));
  for j = 1:size(p0, 2)
    u = rotz(-p0(6, j)) * (pf(1:3, j) - p0(1:3, j));
    if ~p0_is_right_foot(j)
      u(2) = -u(2);
    end
    c(1, j) = abs(u(1) - x_mean) - (x_max - x_min) / 2;
    c(2, j) = abs(u(2) - y_mean) - (y_max - y_min) / 2;
    phi = pf(6, j) - p0(6, j);
    c(3, j) = abs(phi) - biped.max_step_rot;
  end
  c = reshape(c, [], 1);
end


