# Sensor-Data-Fusion-Kalman-Filter

Create your own sensor simulator!
Exercise 4.1
Simulate normally distributed radar measurements!
T = 2 s, 2 radars at r1;2
s = (x1;2
s ; y1;2
s ; z1;2
s )>,
x1;2
s = 0, 100 km, y1;2
s = 100, 0 km, z1;2
s = 10 km.
State at time tk = kT, k 2 Z: xk = (r>k
; _r>k
)>
1. Simulate range and azimuth measurements of the target position rk with a random number
generator normrnd(0; 1) producing normally distributed zero-mean and unit-variance
random numbers:
zp
k =

zr
k
z'
k

=
p
(xk􀀀xs)2+(yk􀀀ys)2+(zk􀀀zs)2􀀀z2
s
arctan( yk􀀀ys
xk􀀀xs
)

+

r normrnd(0;1)
' normrnd(0;1)

with r = 10 m, ' = 0:1 denoting the standard deviations in range and azimuth. Assume
that the radars are not able to measure the elevation angle (see discussion on the
whiteboard!).
2. Transform the measurements in x-y-Cartesian coordinates zrk
(cos z'
k ; sin z'
k )>+rs and
plot them over x-y projection of the true target trajectory! Play with sensor positions and
measurement error standard deviations!
