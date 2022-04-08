
x = float(input("x: "))
y = float(input("y: "))
z = float(input("z: "))
mass = 1

ixx = 0.083 * mass * (y*y + z*z)
iyy = 0.083 * mass * (x*x + z*z)
izz = 0.083 * mass * (y*y + x*x)

print("<ixx>%.10f</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>%.10f</iyy><iyz>0.0</iyz><izz>%.10f</izz>" % (ixx, iyy, izz))
