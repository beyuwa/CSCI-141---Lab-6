import math
from render import InitRender, Render

G = 6.67408e-11

# Define the bodies
central_body = (1e12, (400.0, 400.0), (0.0, 0.0))  # Central body with large mass, positioned at the origin, stationary
planet1 = (1e4, (360.0, 400.1), (0.0001, 1.5))   # Planet 1 starting at (1000, 0) with a velocity vector giving it a circular orbit
planet2 = (1e3, (400.1, 280.0), (-0.5, 0.0001))  # Planet 2 starting at (0, -500) with a velocity vector giving it a circular orbit

# Define the system
system = [central_body, planet1, planet2]


def hyp(x, y):
    return pow((pow(x, 2) + pow(y, 2)),0.5)

def distX(body1, body2):
    return abs(body1[1][0] - body2[1][0])

def distY(body1, body2):
    return abs(body1[1][1] - body2[1][1])

def calculate_distance(body1, body2):
    """Returns the distance between two bodies"""
    pass

    cartesianDist = (distX(body1, body2), distY(body1, body2))
    dist = abs(  pow( ( pow(cartesianDist[0], 2) + pow(cartesianDist[1], 2) ), .5 )  )
    return dist


def calculate_force(body1, body2):
    """Returns the force exerted on body1 by body2, in 2 dimensions as a tuple"""
    forceHyp = G*(body1[0]*body2[0])*pow(calculate_distance(body1,body2), -2)
    #forceHyp / forceX = distHyp / distX(body1, body2)
    forceX = forceHyp * distX(body1, body2) / calculate_distance(body1, body2)
    forceY = forceHyp * distY(body1, body2) / calculate_distance(body1, body2)

    if (body1[1][0] > body2[1][0]): # if body acting on is to the left, negative X
        forceX *= -1
    if (body1[1][1] > body2[1][1]): # if body acting on is ahove, negative Y
        forceY *= -1

    vForce = (forceX, forceY)
    return vForce

def calculate_net_force_on(body, system):
    """Returns the net force exerted on a body by all other bodies in the system, in 2 dimensions as a tuple"""
    
    xdimF = 0
    ydimF = 0

    for i in range(len(system)):
        if system[i] != body:
            xdimF += calculate_force(body, system[i])[0]
            ydimF += calculate_force(body, system[i])[1]
    
    totalF = (xdimF, ydimF)

    return totalF


def calculate_acceleration(body, system):
    """Returns the acceleration of a body due to the net force exerted on it by all other bodies in the system, in 2 dimensions as a tuple"""
    netF = calculate_net_force_on(body, system)
    mass = body[0]
    accelX = pow(mass, -1) * netF[0]
    accelY = pow(mass, -1) * netF[1]
    accel = (accelX, accelY)

    return accel


def get_velocity(body, system, dt):
    accel = calculate_acceleration(body, system)
    accelX = accel[0]
    accelY = accel[1]

    velocX = body[2][0]
    velocY = body[2][1]

    newVelocX = velocX + accelX * dt
    newVelocY = velocY + accelY * dt

    mew = (newVelocX, newVelocY)

    return mew


def update_velocity(system, dt):
    """Updates the velocities of all bodies in the system, given a time step dt"""

    newOrder = []
    
    for i in range(len(system)):
        newOrder += [(system[i][0], system[i][1], get_velocity(system[i], system, dt))]

    return newOrder


def update(system, dt):
    """Update the positions of all bodies in the system, given a time step dt"""
    
    newOrder = []

    for i in range(len(system)):

        velo = update_velocity(system, dt)[i] # the ith body of the system
        velocX = velo[2][0]
        velocY = velo[2][1]

        posX = system[i][1][0] 
        posY = system[i][1][1]

        newPosX = posX + velocX * dt
        newPosY = posY + velocY * dt

        newOrder += [(system[i][0], (newPosX, newPosY), (velocX, velocY))] # reinsertion

    return newOrder


def simulate(system, dt, num_steps):
    """Simulates the motion of a system of bodies for a given number of time steps"""
    statusQuo = system
    for i in range(num_steps):
        statusQuo = update(system, dt)
    
    return statusQuo
    

def simulate_with_visualization(system, dt, num_steps):
    """Simulates the motion of a system of bodies for a given number of time steps, and visualizes the motion"""
    pass

if __name__ == '__main__':
    pass





