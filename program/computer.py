import serial
import struct
import numpy as np

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys

class Vec3:
    def __init__(self,x=0,y=0,z=0):
        self.x,self.y,self.z=(x,y,z)
    def vert(self):
        glVertex3f(self.x,self.y,self.z)
    def __mul__(self,a):
        if a.__class__ == Vec3:
            return a.x*self.x+a.y*self.y+a.z*self.z
        else:
            return Vec3(self.x*a,self.y*a,self.z*a)
    def __add__(self,a):
        return Vec3(self.x+a.x,self.y+a.y,self.z+a.z)
    def norm(self):
        return np.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)
class View:
    def __init__(self):
        self.head=-130
        self.drawVects=True

class Plane:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.M=Vec3()
        self.A=Vec3()
        self.P=Vec3()
        self.G=Vec3()
        self.ledState=False
        self.q=[1,0,0,0]
        self.T=0
       
    def update(self):
        dat=self.ser.readline()[:-1].split(',')
        if dat[0]=='start' and len(dat)>=12:
           # print dat
            try:
                self.A=self.A*0.0+Vec3(*map(float,dat[1:4]))*1.0
                print (self.A.x,self.A.y,self.A.z)
                self.M=self.M*0.0+Vec3(*map(float,dat[4:7]))*1.0
                self.P=self.P*0.9+Vec3(0,0,float(dat[7]))*0.1
                self.T=float(dat[8])
                self.G=self.G*0.0+Vec3(*map(float,dat[9:12]))*1.0
            except ValueError:
                pass
        elif dat[0]=='orient' and len(dat)>=5:
            try:
                self.q=[float(d)/100. for d in dat[1:5]]
            except ValueError:
                pass
        else:
            print dat
    def toggle(self):
        self.ledState=not self.ledState
        if self.ledState:
            self.ser.write('ledOn\n')
        else:
            self.ser.write('ledOff\n')
    def draw(self):
    
        
        glPushMatrix()
        q0,q1,q2,q3=self.q
        glMultMatrixf([ q0**2+q1**2-q2**2-q3**2,  2*(q1*q2-q0*q3),          2*(q1*q3+q0*q2),         0,
                        2*(q1*q2+q0*q3),          q0**2-q1**2+q2**2-q3**2,  2*(q2*q3-q0*q1),         0,
                        2*(q1*q3-q0*q2),          2*(q2*q3+q0*q1),          q0**2-q1**2-q2**2+q3**2, 0,
                        0,                        0,                        0,                       1])
        
        if view.drawVects:
            glDisable(GL_LIGHTING) 
            glLineWidth(2)
            glColor3f(0,1,0)
            glBegin(GL_LINES)
            glVertex3f(0,0,0)
            (self.A*3).vert()
            glEnd()
            glColor3f(1,0,0)
            glBegin(GL_LINES)
            glVertex3f(0,0,0)
            (self.M*3).vert()
            glEnd()
            glColor3f(1,1,0)
            glBegin(GL_LINES)
            glVertex3f(0,0,0)
            (self.G*0.1).vert()
            glEnd()
        glEnable(GL_LIGHTING)    
        color = [1.0,0.,0.,0.8]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glPushMatrix()
        glScalef(4,1,1)
        glutSolidCube(1)
        glPopMatrix()
        
        color = [0,0,1.,0.8]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glPushMatrix()
        glScalef(2,6,0.5)
        glutSolidCube(1)
        glPopMatrix()
        if self.ledState: 
            color = [0,1,0,0.8]
            glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
            glPushMatrix()
            glTranslatef(0.5,-2,0.5)
            glutSolidSphere(0.1,10,10)
            glPopMatrix()
        glPopMatrix()

def initGL():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(400,400)
    glutCreateWindow('Drone Computer User Interface')
    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_POINT_SMOOTH)
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_POLYGON_SMOOTH)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    lightZeroPosition = [10.,4.,10.,0.]
    lightZeroColor = [1.,1.,1.,1.0] #green tinged
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glEnable(GL_LIGHT0)
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(key)
    glutMainLoop()

def key(k,x,y):
    if k=='l':
        plane.toggle()
    if k=='c':
        plane.ser.write('cal\n')
    if k=='v':
        plane.ser.write('calG\n')
    if k=='a':
        view.head+=5
    if k=='d':
        view.head-=5

def display():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    glPushMatrix();
    glRotatef(view.head,0,0,1)
    glDisable(GL_LIGHTING) 
    glLineWidth(2)
    glBegin(GL_LINES)
    for i in range(-10,11):
        glColor3f(0.5,0.1,0)
        glVertex3f(i,-10,3)
        glVertex3f(i,10,3)
        glColor3f(0,0.4,0)
        glVertex3f(-10,i,3)
        glVertex3f(10,i,3)

        glColor3f(1,0.0,1)
        glVertex3f(0,0,0)
        glVertex3f(10,0,0)
    glEnd()

    plane.draw()
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, 400, 400, 0);
    glMatrixMode(GL_MODELVIEW);
    
    glDisable(GL_LIGHTING) 
    
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1, 0, 0, 1);
    glRasterPos2f( 5,5);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Altitude: '+str(plane.P.z)+' m');
    glRasterPos2f( 5,15);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Temperature: '+str(plane.T)+' C');
    glRasterPos2f( 5,25);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Acceleration: '+str(plane.A.norm()));
    glRasterPos2f( 5,35);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Magnetic field: '+str(plane.M.norm()));
    glRasterPos2f( 5,45);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Inclination: '+str( np.arccos(-(plane.M*plane.A)/plane.M.norm()/plane.A.norm())/np.pi*180. ));
    glRasterPos2f( 5,55);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Pitch: '+str( np.arctan(-np.sqrt(plane.A.x**2+plane.A.y**2)/plane.A.z)/np.pi*180. ));
    glPopMatrix()

    glMatrixMode(GL_PROJECTION);
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

    glutSwapBuffers()
    for i in range(10):
        plane.update()
    glutPostRedisplay()

def reshape(width, height):
    glViewport(0, 0, width, height) 
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(40.,(float(width))/height,0.1,400.)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0,16,-4,
               0,0 ,0,
               0,0 ,-1)
    glutPostRedisplay()

def init():
    pass

view=View()
plane=Plane()
plane.update()
init();
initGL()


