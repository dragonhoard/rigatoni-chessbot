function J = rigatoni_Jacobian(q, rigParam)
% find pose dependent 2x2 Jacobian of Rigatoni RR Robot
%   rigParam = [l1, l2, tiny], link lengths and computational neglect term
l1 = rigParam(1);
l2 = rigParam(2);

s1 = sin(q(1));
c1 = cos(q(1));
s12 = sin(q(1) + q(2));
c12 = cos(q(1) + q(2));

J = [(-l1*s1-l2*s12)    -l2*s12;
     (l1*c1+2*c12)       l2*c12];
end


