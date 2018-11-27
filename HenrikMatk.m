format long g

x1 = 500;
x2 = -500;
x3 = 500;
x4 = -500;

distance1 = 1726.648;
distance2 = 1929.952;
distance3 = 1922.728;
distance4 = 1738.344;

d12 = abs(x1) + abs(x2);
d34 = abs(x3) + abs(x4);

a12 = (distance1^2 - distance2^2) / (2 * d12^2) + 1/2;
a34 = (distance3^2 - distance4^2) / (2 * d34^2) + 1/2;

xx = d12 * (2 * a12 - 1);
xy = d34 * (2 * a34 - 1);

xcm = xx/10;
ycm = xy/10;