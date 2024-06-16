%% MAE263B Project 5
% Rotation Joints Only

%clear 
close all
clc

rigatoni_params

%% Forward Kinematics
syms t1 t2 real
%syms l1 l2 d1
% INPUT: DH Table
    % alpha_(i-1)   a_(i-1)   d_i   theta_i   
DHtab = [0, 0, 0, t1; ...
         0, l1, 0, t2;
         0, l2, 0, 0];
%%%%%%%%%%%%%% INPUT END %%%%%%%%%%%%%%%%%%

[Ts, labels] = DH2transforms_lab(DHtab);
N = size(DHtab,1);

T_0_1 = Ts{1,2};
T_0_2 = Ts{2,2};

%% a) Link Inertia Matrix
%syms m1 m2
%syms l1 l2
%syms Ic1x Ic1y Ic1z Ic2x Ic2y Ic2z
%Ic1 = diag([Ic1x Ic1y Ic1z]);
%Ic2 = diag([Ic2x Ic2y Ic2z]);
%Ic1 = m1*l1^2/12*diag([0 1 1]); % TODO
%Ic2 = m2*l2^2/12*diag([0 1 1]); % TODO

R_0_1 = rot(T_0_1);
R_0_2 = rot(T_0_2);

I_0_1 = R_0_1*Ic1*transpose(R_0_1);
I_0_2 = R_0_2*Ic2*transpose(R_0_2);

%% b) Link Jacobian Matrix
P_0_1 = pos(T_0_1);
P_0_2 = pos(T_0_2);

% position of center of mass
%syms P_1c1x P_2c2x
%P1c1 = [P_1c1x; 0; 0];
%P2c2 = [P_2c2x; 0; 0];
%P1c1 = [l1/2; 0; 0];
%P2c2 = [l2/2; 0; 0];

% position of center of mass in base frame
P_0_c1 = T_0_1*[P1c1;1];
P_0_c1 = P_0_c1(1:3);
P_0_c2 = T_0_2*[P2c2;1];
P_0_c2 = P_0_c2(1:3);

% find Jv Jw
Z_0_1 = T_0_1(1:3,3);
Z_0_2 = T_0_2(1:3,3);
Jv1 = simplify([cross(Z_0_1, P_0_c1-P_0_1), cross(Z_0_1, P_0_c1-P_0_2)]);
Jw1 = [Z_0_1, Z_0_2];

Z_0_2 = T_0_2(1:3,3);
Jv2 = simplify([cross(Z_0_1, P_0_c2-P_0_1), cross(Z_0_2, P_0_c2-P_0_2)]);
Jw2 = [Z_0_1, Z_0_2];

%% c) Manipulator Inertia Matrix
M = transpose(Jv1)*m1*Jv1 + ...
    transpose(Jw1)*I_0_1*Jw1 + ...
    transpose(Jv2)*m2*Jv2 + ...
    transpose(Jw2)*I_0_2*Jw2;
M = simplify(M);

%% d) Velocity Coupling Vector
syms t1d t2d real
q = [t1 t2];
qd = [t1d t2d];

V = sym([0; 0]);
for i = 1:2
    for j = 1:2
        for k = 1:2
            V(i) = V(i) + ...
                simplify((diff(M(i,j), q(k)) - diff(M(j,k), q(i))/2)*qd(k)*qd(j));
        end
    end
end

%% e) Gravitational Vector
syms g
gvec = [0; 0; 0];

G1 = simplify(-m1*transpose(gvec)*Jv1(:,1) - m2*transpose(gvec)*Jv2(:,1));
G2 = simplify(-m1*transpose(gvec)*Jv1(:,2) - m2*transpose(gvec)*Jv2(:,2));

%% construct tau
syms t1dd t2dd real
qdd = [t1dd t2dd];
tau1 = M(1,1)*qdd(1) + M(1,2)*qdd(2) + V(1) + G1
tau2 = M(2,1)*qdd(1) + M(2,2)*qdd(2) + V(2) + G2

