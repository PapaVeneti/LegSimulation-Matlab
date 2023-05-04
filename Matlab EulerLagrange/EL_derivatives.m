function [L_qt,L_q] = EL_derivatives(L,states,N)

qs  = [eye(N) ,zeros(N)]* states; %angles
qst = [zeros(N),eye(N) ]* states; %angular velocites

t = symvar(qs);

L_q       = jacobian(L,qs);
L_qt_temp = jacobian(L,qst);
L_qt      = diff(L_qt_temp,t);

L_q = simplify(L_q.');
L_qt = simplify(L_qt.');
