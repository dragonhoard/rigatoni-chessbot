function B = Bfcn(t1,t2)
%Bfcn
%    B = Bfcn(T1,T2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    08-May-2024 23:46:10

t3 = cos(t2);
t4 = t3.*6.80625e-3;
t5 = t4+6.2390625e-3;
B = reshape([t3.*1.36125e-2+1.947543323555957e-2,t5,t5,6.2390625e-3],[2,2]);
end
