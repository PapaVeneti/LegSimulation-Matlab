xc   = [                   0.16763
                   0.07763
                       0.4];
tw = [1 2 3 4 5];
x(1,1:5) = 0.16763 ; 
% x(1,1:10) = 0.16763 ; 

x(2:3,1) = [0.4;0.3 ] ; norm(x(:,1)-xc)  
x(2:3,2) = [0.2;0.2 ] ;  norm(x(:,2)-xc)
x(2:3,3) = [0.1;0.1 ]  ; norm(x(:,3)-xc)
x(2:3,4) = [0.05;0.045/2+0.005 ]  ; norm(x(:,4)-xc)


x(2:3,5) = [0.0;0.045/2 ]  ; norm(x(:,5)-xc)
% x(2:3,6) = [0.1;0.18 ]  ; norm(x(:,6)-xc)
% x(2:3,7) = [0.1;0.15 ]  ; norm(x(:,7)-xc)
% x(2:3,8) = [0.1;0.18 ]  ; norm(x(:,8)-xc)
% x(2:3,9) = [0.1;0.13 ]  ; norm(x(:,9)-xc)
% x(2:3,10) = [0.1;0.1 ]  ; norm(x(:,10)-xc)


