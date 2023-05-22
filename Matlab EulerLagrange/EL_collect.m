function [B,C,G] = EL_collect(L_qt,L_q,states,N)
% This functions collects the terms of the euler lagrange equations in
% order to create the dynamics matrices. 
%Input:
%-L_qt as a `sumfun` : L_qt(states(t)),
%-L_q as a `sumfun`  : L_q(states(t)),
%-states as a `symfun` : states(t)
%-Dofs
%
%Output: 
%-Mass matrix as a `sym` matrix
%-Coriolis and Centrifugal matrix as a `sym` matrix
%-Gravity vector as a `sym` matrix

syms T

qs  = [eye(N) ,zeros(N)]* states; %angles
qst = [zeros(N),eye(N) ]* states; %angular velocites

t = symvar(qs);

%initialization
B = sym('B',N);
C = sym('C',N);
G = sym('G',[N,1]);

for r=1:N
    for c=1:N
        B(r,c) = 0;
        C(r,c) = 0;
    end
    G(r)=0;
end

%can only index using matrix multiplication: -> So we define selectors
selectR = zeros(1,N);
selectC = zeros(1,N);

%% Get B matrix

for r=1:N %row -> TK_qrt
    %set row selector:
    selectR(r) = 1;
    L_qt_r = selectR*L_qt;

    for c=1:N %column -> collect qc_tt
    %set column selector:
    selectC(c) = 1;
    diff_c = diff( selectC*qst ,t);

    %get the coefficients of the qi_tt = q_i,doubledot:
    TEMP_COEFFS = coeffs(L_qt_r, diff_c ,'All' ); % expecting two coeffs, as qtt is in first order;
    NC = size(TEMP_COEFFS(T),2); %number of coefficients
    if NC==2
        B(r,c) = TEMP_COEFFS*[1;0]; %extract b11
    end 
    
    L_qt_r = L_qt_r - B(r,c) * diff_c ; % we remove the term we just collected
    L_qt_r = simplify(L_qt_r);

    %reset selectors
    selectC(c) = 0;
    end

    selectR(r) = 0;
end

L_qt = L_qt - B*diff(qst,t); % we remove the acceleration terms

%% Get C matrix

Lres = L_qt- L_q; %residual Lagrangian

%L_q = L_q(qs,diff(qs,t),qst) but diff(qs,t) = qst. So we have to subs qst
Lres =subs(Lres,diff(qs,t),qst);
Lres = simplify(Lres);

for r=1:N %row -> L_qt(r)+L_q(r)
    %set row selector:
    selectR(r) = 1;
    Lres_r = selectR*Lres;

    for c=1:N %column -> collect qc_tt

    %set column selector:
    selectC(c) = 1;
    qt_c = selectC*qst ;

    TEMP_COEFFS = coeffs(Lres_r , qt_c ,'All'); % expecting three coeffs
    NC = size(TEMP_COEFFS(T),2); %number of coefficients
    if NC==2
        C(r,c) = TEMP_COEFFS*[1;0];
    elseif NC==3
        C(r,c) = TEMP_COEFFS*[1;0;0]*qt_c; %second order term
        C(r,c) = TEMP_COEFFS*[0;1;0] +  C(r,c); %first order term
    end 
    Lres_r = Lres_r - C(r,c) *qt_c;
    Lres_r = simplify(Lres_r);

    %reset selectors
    selectC(c) = 0;
    end

    selectR(r) = 0;
end


%% Get G matrix
Gtemp =  Lres - C*qst;

for r =1:N
    selectR(r)=1;
    
    G(r) = selectR*Gtemp;

    selectR(r)=0;
end

%% Output
B = simplify(B);
C = simplify(C);
G = simplify(G);


