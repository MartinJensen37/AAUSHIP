

#Transformation matrix from b to n
Tn = np.array([cos(z), -sin(z), x; sin(z), cos(z), y; 0, 0, 1])

#Vector with the desired postions of the boat in NED frame
Pd = np.array([xdn; ydn; 1])
# The inverse of Tb
Tinv = np.linalg.inv(Tn)

#Vector with desired postions in boat frame
Tb = Tinv*Pd

#Output of the transformation matrix
xdb = Tn(0)

ydb = Tn(1)

zdb = zd - z