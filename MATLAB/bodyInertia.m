clear all;
mass = [81 36 37 2*33 43]*1e-3;
radius =[60 110 100 30 80]*1e-3;
width = [16 12 10 23 8]*1e-3;
height = [30 90 100 31 173]*1e-3;
I0 = zeros(5,1);

for i = 1:length(I0)
    I0(i) = (mass(i) * ( width(i)^2 + height(i)^2))/12 + mass(i)*radius(i)^2;
end

Ibody = sum(I0);
disp(Ibody);