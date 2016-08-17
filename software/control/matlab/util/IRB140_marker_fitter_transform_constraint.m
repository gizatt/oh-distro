function f = IRB140_marker_fitter_transform_constraint(x, fk, mk)
    lhs = fk(1:3, 1:3) * x(7:9) + fk(1:3, 4);
    rhs = rpy2rotmat(x(4:6))*mk + x(1:3);

    f = (lhs - rhs).' * (lhs-rhs)
end