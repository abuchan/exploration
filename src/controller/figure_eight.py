#!/usr/bin/python

import numpy
from scipy.special import jn_zeros
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def update(state,u,dt,arc=True):
  v,a = u
  _,_,theta = state
  rotation = numpy.array(
    [[numpy.cos(theta), -numpy.sin(theta)],
     [numpy.sin(theta), numpy.cos(theta)]])
  d_theta = dt * a
  
  if arc:
    if (a == 0.0):
      delta = numpy.array([dt*v, 0.0, 0.0])
    else:
      r = v/a
      delta = numpy.array([r * numpy.sin(d_theta), r * (1.0-numpy.cos(d_theta)), d_theta])
  else:
    delta = numpy.array([dt*v, 0.0, d_theta])
  
  delta[0:2] = rotation.dot(delta[0:2])
  
  return state + delta

# control - function that takes a state vector and produces a linear/angular velocity command
# t - final time
# dt - time step
# arc - if true, simulate trajectory as piece-wise arcs (more accurate), 
#   otherwise just use straight line
def trajectory(control, s0=[0,0,0], t=60.0, dt=0.01, arc=True):

  t = numpy.arange(0.0,t,dt)

  state = numpy.zeros((len(t),3))
  state[0] = s0

  u = numpy.zeros((len(t),2))

  for i in range(1,len(t)):
    u[i-1] = control(state[i-1],t[i-1])[0]
    state[i] = update(state[i-1],u[i-1],dt,arc)

  return state, u, t

def sine_angular_control(s, t, v=1.0):
  # Amplitude is the first root of the 0-th order Bessel function
  A = jn_zeros(0,1)[0]

  v = v*numpy.ones(t.shape)
  a = A*numpy.sin(t)

  return numpy.vstack([v,a]).T
 
def sine_position_control(s, t, v=1.0):
  v = v*(numpy.cos(t)**2+4*numpy.cos(2*t)**2)**0.5
  a = 4*numpy.sin(t)*(numpy.cos(2*t)+2)/(numpy.cos(2*t)+4*numpy.cos(4*t)+5)
  
  return numpy.vstack([v,a]).T

def attractor_control(s, t, v=1.0):
  f0 = [1.0,0.0]
  f1 = [-1.0,0.0]

  r = 1.0
  k = 10.0
  if len(s.shape) > 1:
    dist = (s[:,0:2]**2).sum(1)
  else:
    dist = (s[0:2]**2).sum()

  a = 2*numpy.pi + k*(r-dist)
    
  return numpy.vstack([v,a]).T

# s is [x,y,theta], w is lemniscate width, v is desired velocity on lemniscate
def lemniscate_vector(state, w=0.5):
  x,y,theta = state

  # radius squared, width squared
  r2 = x**2 + y**2
  w2 = w**2

  # gradient vector towards lemniscate trajectory, normalized to have max val of 1 at foci
  down = -1.0/(w2**2)*(r2**2 - w2*(x**2-y**2)) * numpy.array([2*r2*x - w2*x, 2*r2*y + w2*y])

  # weight vector more if we are inside the lemniscate
  if r2**2 - w2*(x**2-y**2) <= 0.0:
    down *= 10.0

  norm = sum(down**2)**0.5
  if norm != 0.0:
    down = (1.0-10.0**(-norm))*(down/norm)

  # if we are above the lemniscate, use the trig definition of the gradient 
  if y >= 0.0:
    angle = 3.0*numpy.arctan2(y,x) + numpy.pi/2
    around = numpy.array([numpy.cos(angle),numpy.sin(angle)])

  # otherwise use normalized lemniscate direction
  else:
    # vector moving around in the direction of the trajectory
    dx = (w2 + 2*r2)*y
    dy = (w2 - 2*r2)*x
    around = -numpy.array([dx,dy])/((dx**2+dy**2)**0.5)

  # mirror around field in left-hand plane
  around *= numpy.sign(x)
  
  # if we are near the center, mix in fixed angle drive
  # cr2 = radius at which mixing begins squared, in fractions of w
  cr2 = 0.1**2

  if r2 < cr2*w2:
    center = 2**0.5/2*numpy.array([1.0,-1.0])
    if numpy.cos(theta) < 0.0:
      center[0] *= -1

    around = ((cr2*w2-r2)/(cr2*w2) * center) + (r2/(cr2*w2) * around)

  # return weighted sum of gradient descent and rotational vector
  return around + 10*down

def threshold_gain(x, gain=1.0, bounds=[-numpy.inf,numpy.inf], offset=0.0):
  val = gain*(x-offset)
  if val > bounds[1]:
    val = bounds[1]

  if val < bounds[0]:
    val = bounds[0]

  return val 

def lemniscate_control(s, t, v=0.1, w=0.5):
  vector = lemniscate_vector(s,w)
  _,_,theta = s
  theta_vf = numpy.arctan2(vector[1],vector[0])
  
  theta_diff = (theta_vf - theta) % (2*numpy.pi)
  if theta_diff > numpy.pi:
    theta_diff -= 2*numpy.pi

  v_cmd = threshold_gain(
    (sum(vector**2)**0.5)*numpy.cos(theta_diff),
    gain=v,bounds=[0.0,2*v]
  )
  
  a_cmd = threshold_gain(theta_diff,gain=50.0,bounds=[-3.0,3.0])
 
  return numpy.vstack([v_cmd,a_cmd]).T

def plot_lemniscate(w=0.5,nt=200, ax_in=None):
  t = numpy.linspace(0,2*numpy.pi,nt)
  x = w*numpy.cos(t)/(1+numpy.sin(t)**2)
  y = w*numpy.sin(t)*numpy.cos(t)/(1+numpy.sin(t)**2)
  
  if ax_in is None:
    ax = plt.gca()
  else:
    ax = ax_in
  ax.plot(x,y,'k')
  plt.axis('equal')
  plt.grid(True)

  if ax_in is None:
    plt.show()

def plot_trajectory(state, is_2d=True, ax_in=None):
  if is_2d:
    if ax_in is None:
      ax = plt.gca()
    else:
      ax = ax_in
    ax.plot(state[:,0],state[:,1],'b')
  else:
    if ax_in is None:
      fig = plt.figure()
      ax = fig.add_subplot(111, projection='3d')
    else:
      ax = ax_in
    ax.plot(state[:,0],state[:,1],state[:,2])
    ax.set_zlabel('theta')

  plt.xlabel('x')
  plt.ylabel('y')
  plt.axis('equal')
  plt.grid(True)
  
  if ax_in is None:
    plt.show()

def plot_time_trajectory(t, state, u=None):
  plt.plot(t,state[:,0],label='x')
  plt.plot(t,state[:,1],label='y')
  plt.plot(t,state[:,2],label='theta')
  if u is not None:  
    plt.plot(t,u[:,0],label='v')
    plt.plot(t,u[:,1],label='a')
  plt.xlabel('time')
  plt.legend()
  plt.grid(True)
  plt.show()
  
def evaluate_vector(vector_function, bounds = [[-1.5,1.5],[-0.6,0.6]], step=0.1,theta=0.0):
  x_range = numpy.arange(bounds[0][0],bounds[0][1],step)
  y_range = numpy.arange(bounds[1][0],bounds[1][1],step)
  X,Y = numpy.meshgrid(x_range,y_range)
  U,V = numpy.zeros(X.shape),numpy.zeros(X.shape)
  for i in range(len(X)):
    for j in range(len(X[0])):
      U[i,j],V[i,j] = vector_function([X[i,j],Y[i,j],theta])
  return X,Y,U,V

def plot_vector(vector_function, bounds = [[-1.5,1.5],[-0.6,0.6]], step=0.1,theta=0.0,ax_in=None):
  if ax_in is None:
    ax = plt.gca()
  else:
    ax = ax_in
  ax.quiver(*evaluate_vector(vector_function,bounds,step,theta),
    units='xy',angles='xy',width=0.001
  )
  ax.axis('equal')
  
  if ax_in is None:
    plt.show()

def plot_vector_control(vector_function, state, ax_in=None):
  substate = state[::10,:]
  vector_control = numpy.array([vector_function(s) for s in substate])

  if ax_in is None:
    ax = plt.gca()
  else:
    ax = ax_in

  ax.quiver(substate[:,0],substate[:,1],vector_control[:,0],vector_control[:,1],
    units='xy',angles='xy',width=0.0002,color='r'
  )
  
  ax.axis('equal')

  if ax_in is None:
    plt.show()

# v - average velocity of the robot on the trajectory in m/s
# w - width of the lemniscate trajectory in m
# s0 - initial condition of robot as  [x,y,theta] in m,m,rad
# t - simulation time in seconds
# dt - time step
# bounds - bounds of graph [[minx,maxs],[miny,maxy]]. if left None autoscales
#   based on trajectory.
# step - delta between quiver plot points
# theta - static theta used to render quiver plot
# ax_in - axes on which to plot graph
def plot_all(v=0.1,w=0.5,s0=[0.0,0.0,0.0],t=60.0,dt=0.01,bounds=None, step=None,theta=0.0,ax_in=None):
  if ax_in is None:
    ax = plt.gca()
  else:
    ax = ax_in
  
  lc = lambda s,t: lemniscate_control(s,t,v=v,w=w)

  state, u, t = trajectory(lc,s0=s0,t=t,dt=dt)
  ax = plt.gca()
  plot_lemniscate(w=w,ax_in=ax)

  if bounds is None:
    mins,maxs = state.min(0),state.max(0)
    boundary = 0.05*(maxs-mins)
    bounds = [
      [mins[0]-boundary[0],maxs[0]+boundary[0]],
      [mins[1]-boundary[1],maxs[1]+boundary[1]]
    ]
  
  if step is None:
    min_range = min(bounds[0][1]-bounds[0][0],bounds[1][1]-bounds[1][0])
    step = min_range/50.0
  
  lv = lambda s: lemniscate_vector(s,w=w)
  
  plot_vector(lv, bounds, step, theta, ax_in=ax)
  
  plot_trajectory(state,ax_in=ax)
  
  plot_vector_control(lv,state,ax_in=ax)
  
  ax.axis([bounds[0][0],bounds[0][1],bounds[1][0],bounds[1][1]])

  plt.title('s0=%s, v=%f, w=%f' % (str(s0),v,w))

  if ax_in is None:
    plt.show()

if __name__ == '__main__':
  plot_trajectory(trajectory(sine_angular_control))

