clc, clear;

syms q1 q2 q3 l0 l1 l2 l3;

%1
% theta, alpha, a, d
l00 = [0 pi/2 0 l0];
A00 =DH_HTM(l00, 'r');

l01 = [q1 -pi/2 0 0];
A01 =DH_HTM(l01, 'r');

l12 = [q2 0 l2 l1];
A12 =DH_HTM(l12, 'r');

l2E = [q3 0 l3 0];
A2E = DH_HTM(l2E, 'r');

%2
A00;
A01;
A12;
A2E;

%3
A01 = A00 * A01;
A02 = A01*A12;
A03 = A02*A2E;

b = [0;0;1];
b0 = [1;0;0];
b1 = A01(1:3,1:3)*b;
b2 = A02(1:3,1:3)*b;

r = [0;0;0;1]
r0E = A03*r-r;
r1E = A03*r-A01*r;
r2E = A03*r-A02*r;


r0E = r0E(1:3);
r1E = r1E(1:3);
r2E = r2E(1:3);

JL1 = cross(b0,r0E);
JL2 = cross(b1,r1E);
JL3 = cross(b2,r2E);

JL = simplify([JL1 JL2 JL3]);
JA = [b0 b1 b2];

%4
J = simplify([JL; JA])

detJL = det(JL)

p0E = A03(1:3,4)

px = p0E(1)
py = p0E(2)
pz = p0E(3)



