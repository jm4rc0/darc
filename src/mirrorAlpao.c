//Alpao mirror, using Ethernet interface adapter.
//Including creep compensation algorithms.

/*
Creep compensation:

1.  Need a record of the average DM shape over the last few(?) hours.
a.  Probably using a gain/decay factor - with a very low gain.
b.  What if DM is frozen (ie no actuator buffer active)?
c.  What about an initial starting state?
d.  How long should the average shape be averaged over?
e.g. if AO loop at 150Hz, create average shape by doing Av = (1-g)Av + gNew where g = 1e-6, or something.
f.  Update this mean shape every few (X) minutes.

2.  Once have the average shape, then need the creep compensation.  Decays gradually towards the average shape.
a.  


 */
