function Ax = exterior(Av)
if isnumeric(Av)
Ax = zeros(3);
else
Ax= sym('Ax',3);
Ax(1,1) = 0; Ax(2,2) =0; Ax(3,3)= 0;
end

if class(Av) == "symfun"
    x = [1 0 0]*Av;
    y = [0 1 0]*Av;
    z = [0 0 1]*Av;
else
    x = Av(1);
    y = Av(2);
    z = Av(3);

end

Ax(1,2) = -z;Ax(2,1) =  z;
Ax(1,3) =  y;Ax(3,1) =  -y;
Ax(2,3) = -x;Ax(3,2) =  x;
end