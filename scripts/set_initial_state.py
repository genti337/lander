import numpy
import math
import liblander

# Default Moon Parameters
moon_radius = 1.74e6
moon_mass = 7.35e22
gm = 6.673e-11

def set_trans_state(dyn_body, alt=0.0, long_asc=0.0, inclination=0.0, arg_peri=0.0, true_anom=0.0, frame='orbital'):
   eci_pos = [0.0, 0.0, 0.0]

   if frame == 'inertial':
      pass
   elif frame == 'orbital':
      # Radius is assumed to be the Lunar Radius plus the Altitude
      r = alt + moon_radius

      # Circular Orbital Velocity
      vel_mag = math.sqrt((gm * moon_mass) / r)

      # Rotation from Perifocal to Inertial Reference Frame
      x1 = math.cos(long_asc) * math.cos(arg_peri) - math.sin(long_asc) * math.cos(inclination) * math.sin(arg_peri)
      x2 = math.sin(long_asc) * math.cos(arg_peri) + math.cos(long_asc) * math.cos(inclination) * math.sin(arg_peri)
      x3 = math.sin(inclination) * math.sin(arg_peri)
      y1 = -math.cos(long_asc) * math.sin(arg_peri) - math.sin(long_asc) * math.cos(inclination) * math.cos(arg_peri)
      y2 = -math.sin(long_asc) * math.sin(arg_peri) + math.cos(long_asc) * math.cos(inclination) * math.cos(arg_peri)
      y3 = math.sin(inclination) * math.cos(arg_peri)
      z1 = math.sin(inclination) * math.sin(arg_peri)
      z2 = -math.sin(inclination) * math.cos(arg_peri)
      z3 = math.cos(inclination)
      T_eci2perifocal = numpy.array([[x1, x2, x3], [y1, y2, y3], [z1, z2, z3]])
      T_perifocal2eci = numpy.transpose(T_eci2perifocal)

      # Position/Velocity of the Vehicle wrt Perifocal Frame
      perifocalPos = [(moon_radius + alt) * math.cos(true_anom), (moon_radius + alt) * math.sin(true_anom), 0.0]
      perifocalVel = [-vel_mag * math.sin(true_anom), vel_mag * math.cos(true_anom), 0.0]

      # Inertial Position/Velocity
      eciPos = numpy.dot(T_perifocal2eci, perifocalPos)
      eciVel = numpy.dot(T_perifocal2eci, perifocalVel)

      # Set the Dynamic Body Position/Velocity
      dyn_body.setECIPos(eciPos[0], eciPos[1], eciPos[2])
      dyn_body.setECIVel(eciVel[0], eciVel[1], eciVel[2])
   elif frame == 'lvlh':
      pass

def set_rot_state(dyn_body, frame='lvlh', euler=[0.0, 0.0, 0.0]):

   if frame == 'lvlh':
      dyn_body.setLVLHAtt(euler[0], euler[1], euler[2])
   else:
      pass
