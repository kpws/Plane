import serial
import struct

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
        return Vec3(self.x*a,self.y*a,self.z*a)
    def __add__(self,a):
        return Vec3(self.x+a.x,self.y+a.y,self.z+a.z)

class Plane:
    def __init__(self):
        #self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.M=Vec3()
        self.A=Vec3()
        self.P=Vec3()
        self.G=Vec3()
        self.ledState=False
       
    def update(self):
        dat=self.ser.readline()[:-1].split(',')
        if dat[0]=='start' and len(dat)>=12:
            #print dat
            try:
                self.A=self.A*0.0+Vec3(*map(int,dat[1:4]))*1.0
                self.M=self.M*0.0+Vec3(*map(int,dat[4:7]))*1.0
                self.P=self.P*0.0+Vec3(0,0,float(dat[7]))*1.0
                self.T=float(dat[8])
                self.G=self.G*0.0+Vec3(*map(int,dat[9:12]))*1.0
            except ValueError:
                self.update()
        else:
            print dat
            self.update()
    def toggle(self):
        self.ledState=not self.ledState
        if self.ledState:
            self.ser.write('ledOn\n')
        else:
            self.ser.write('ledOff\n')
    def draw(self):
        glDisable(GL_LIGHTING) 
        glLineWidth(2)
        glColor3f(0,1,0)
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        (self.A*0.004).vert()
        glEnd()
        glColor3f(1,0,0)
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        (self.M*0.01).vert()
        glEnd()
        
        glColor3f(1,1,0)
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        (self.G*0.0001).vert()
        glEnd()
        
        glEnable(GL_LIGHTING)    
        color = [1.0,0.,0.,0.8]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glPushMatrix()
        glScalef(1,4,1)
        glutSolidCube(1)
        glPopMatrix()
        
        color = [0,0,1.,0.8]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glPushMatrix()
        glScalef(6,2,0.5)
        glutSolidCube(1)
        glPopMatrix()
        if self.ledState: 
            color = [0,1,0,0.8]
            glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
            glPushMatrix()
            glTranslatef(0.5,-2,0.5)
            glutSolidSphere(0.1,10,10)
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

def display():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

    plane.draw()
        
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, 400, 400, 0);
    glMatrixMode(GL_MODELVIEW);
    
    glDisable(GL_LIGHTING) 
    
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f( 5,30);
    glColor4f(0, 0, 1, 1);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Altitude: '+str(plane.P.z)+' m');
    glRasterPos2f( 5,60);
    glutBitmapString(GLUT_BITMAP_9_BY_15, 'Temperature: '+str(plane.T)+' C');
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
    gluLookAt(-8,16,4,
               0,0 ,0,
               0,0 ,1)
    glutPostRedisplay()


plane=Plane()
plane.update()
initGL()


