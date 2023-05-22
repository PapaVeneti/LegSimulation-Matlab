%This scripts calculates the transformation matrices for the leg

calcs = false;  
% true if i want to double check c++ code, or get EOM
% false to get the symbolic matrices for kinematics

%% Definitions:
syms q1(t) q2(t) q3(t);
assume(q1(t),'real');
assume(q3(t),'real');
assume(q2(t),'real');

if calcs
   LB0 = 0.05555;
   L01 = 0.07763;
   L12 = 0.11208;
   L23 = 0.2531;
   L3E = 0.223;
   H = 0.4;
else
    syms H LB0 L01 L12 L23 L3E real
end

%% Matrices
TW = [
    0 -1 0 LB0;
    0 0 -1 0;
    1 0 0 H;
    0 0 0 1];

T1 = [
    cos(q1) -sin(q1) 0 0 ;
    sin(q1) cos(q1) 0 0;
    0 0 1 -L01;
    0 0 0 1];

T2 = [
    sin(q2) cos(q2) 0 0 ;
    0 0 -1 -L12;
    -cos(q2) sin(q2)  0 0 ;
    0 0 0 1];

T3 = [
    cos(q3) -sin(q3) 0 L23 ;
    sin(q3) cos(q3) 0 0;
    0 0 1 0 ;
    0 0 0 1];

TE = [
    1 0 0 L3E;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

%% Matrices of interest
T123  = simplify( T1*T2*T3 );
T123E = simplify( T123*TE  );
TWE   = simplify( TW*T123E );





