
def cylinder(m, r, h):
    Izz = 0.5 * m * r**2
    Ixx = (1.0/12.0) * m * (3*r**2 + h**2)
    Iyy = Ixx
    return Ixx, Iyy, Izz

print cylinder(0.05, 0.05, 0.01)

