#include <Arduino.h>
#include <Wire.h>
#include <LPS331.h>
#include <LSM303.h>
#include <L3G.h>

#define COMMUNICATE
//#define CAL

#include "orientation.c"
#include "ServoFast.h"
#include "NewPing.h"

//#include "svdcmp.c"
//#include "magfield.c"

const float SOUND_SPEED = 340.3;
const float GRAV_ACC = 9.82;

const int WARM_UP=0;
const int CALIBRATE=1;
const int READY=3;
const int MAX_CP=16;
const int CAL_MEANS=20;
//const int N_SUN_SENSORS=8;
const float GYRO_SENS=17.5e-3*PI/180.;
const int GREEN_LED = 13;
const int LEFT_PHOTO = 6;
const int RIGHT_PHOTO = 7;
const int SONAR_TRIG = 24;
const int SONAR_ECHO = 26;

const float phi=12./180.*PI;
const float roll[4] = {-0.67125, -0.222314, -0.222314, -0.67125};
const float pitch[4] = {0.402761, 0.581192, -0.581192, -0.402761};
const float Ixx=0.053389;
const float Iyy=0.00649196;
const float CM[3]={-0.041,0.,0.};
const float accelerometerPos[3]={-0.1,0.05,0.};
const float rollEfficiency=0.0102422/Ixx;// torque at full deflection/q/moment of inertia I_{xx} 
const float pitchEfficiency=0.00663437/Iyy;//torque at full deflection/q/moment of inertia I_{yy} 

const float P=100.;
const float I=0;//300.;
const float D=0.3;
const float integrateMax=0.3/I;
const float maxCSus=50.*1400./165.; //Max 50 degrees control surface deflection, 500 microseconds/90 degrees
const float uMax=1.5;
const float landAngle=-0./180*PI;
const float MAX_SONAR_DIST=1.0;

const float cosHalfLandAngle=cos(landAngle/2.);
const float sinHalfLandAngle=sin(landAngle/2.);
const float accelerometerRelPos[3]={accelerometerPos[0]-CM[0],accelerometerPos[1]-CM[1],accelerometerPos[2]-CM[2]};
const float cosPhi=cos(phi);
const float sinPhi=sin(phi);

Servo servos[4];

LPS331 ps;
LSM303 compass;
L3G gyro;

NewPing sonar(SONAR_TRIG, SONAR_ECHO, int(MAX_SONAR_DIST*100.) );

int first;
int mode;
long t,t2;
unsigned long lmi;
float dt;
unsigned int us[3];

float calPoint[MAX_CP][10];
int calPoints=0;
float calVal[2][6];
float Gc[3];

float longitude,latitude,altitude;
float Be[3],Ae[3];//,Se[3];

float B[3],A[3],G[3];//,S[3];
//int sunSens[N_SUN_SENSORS];
float pressure,psAltitude,psTemperature;
unsigned int sonar_us, sonar_us_vol;

float x[3];//Kahlman filter state, [xpos, xvel, xacc, ypos, yvel, ...]
float P_priori[3][3];
float P_posteriori[3][3];
float q[4];
float qw[4];

float i_roll;
float i_pitch;

float u_roll;
float u_pitch;
float u_skew;

#ifdef CAL
#define LISTEN
#define COMMUNICATE
#include "qr_fact.c"
#endif

#include "calibrate.cpp"

void setup()
{
    pinMode(GREEN_LED, OUTPUT);
#ifdef COMMUNICATE
    Serial.begin(115200);
#endif
   /* for(float i=80;i<1500.;i*=1.1)
    {

        tone(8,i,30);
        delay(30);
        tone(8,i*2,30);
        delay(30);
    }*/

    /*servos[0].attach(10);
    servos[1].attach(9);
    servos[2].attach(12);
    servos[3].attach(11);
    */
    for(int i=0;i<4;i++)
    {
        servos[i].writeMicroseconds(1500);
    }
    delay(1000);

    Wire.begin();

    ps.init();
    ps.enableDefault();
    ps.writeReg(LPS331_RES_CONF, 0b11111010);

    compass.init();
    compass.enableDefault(); 

    gyro.init();
    gyro.enableDefault();
    gyro.writeReg(L3G_CTRL_REG4, 0b00010000);//500 degrees/sec max

    //get these from GPS
    longitude=(15.638586/180.)*PI;
    latitude=(58.393998/180.)*PI;
    altitude=100;
    //julianDate=yymmdd_to_julian_days(2013,7,22);
    //float field[6];
    //SGMagVar(latitude,longitude,altitude/1000.,julianDate,11,field);

    //./geomag70.exe IGRF11.COF 2013.5 D M0.0 58.4000 15.6167
    Be[0]=  15741.8;
    Be[1]= 1182.2; //nT, Liljegatan, Linköping, 2013.5, IGRF11
    Be[2]= 48459.7;
    float Bem=sqrt(Be[0]*Be[0]+Be[1]*Be[1]+Be[2]*Be[2]);
    for(int i=0;i<3;i++)
        Be[i]/=Bem; 

    Ae[0]=0.;
    Ae[1]=0.;
    Ae[2]=-1.;

    x[0]=1.5;
    x[1]=0.;
    x[2]=0.;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            P_priori[i][j]=0.0;
            P_posteriori[i][j]=0.0;
        }

    q[0]=1.;
    q[1]=0.;
    q[2]=0.;
    q[3]=0.;

    qw[0]=1.;
    qw[1]=0.;
    qw[2]=0.;
    qw[3]=0.;

    i_roll=0.;
    i_pitch=0.;
    
    lmi=0;

    mode=CALIBRATE;
    initCal();

    first=100;
    t=0;
    t2=0;
    dt=0;

    sonar_us=0;
    sonar_us_vol=0;
}

void echoCheck()
{
    if(sonar.check_timer())
    {
        sonar_us_vol=sonar.ping_result;
    }
}

void loop()
{
    /////////////////LISTEN FOR SERIAL DATA///////////////////////
#ifdef LISTEN
    const int bufLen=32;
    char buf[bufLen];
    if(Serial.available())
    {
        if(Serial.readBytesUntil('\n', buf, bufLen))
        {
            if(strncmp(buf,"ledOn",5)==0)
            {
                digitalWrite(GREEN_LED, HIGH);
            }
            if(strncmp(buf,"ledOff",6)==0)
            {
                digitalWrite(GREEN_LED, LOW);
            }
            else if(strncmp(buf,"cal",3)==0)
            {
                if(mode==CALIBRATE)
                {
                    cal();
                }
            }
        }
    }
#endif

    //////////////////////////////////READ SENSORS///////////////////////
    compass.read();
    gyro.read();

    pressure = ps.readPressureMillibars();
    psTemperature = ps.readTemperatureC();

    unsigned long mi=micros();
    dt=((mi-lmi)*1e-6);
    lmi=mi;


    /////////////////////////////CONVERT SENSORS VALUES TO PHYSICAL QUANTITIES//////////////////
    float Lx,Ly,Lz;

    Lx=((float)compass.a.x-calVal[0][0])/calVal[0][3];
    Ly=((float)compass.a.y-calVal[0][1])/calVal[0][4];
    Lz=((float)compass.a.z-calVal[0][2])/calVal[0][5];
    A[0] = -Lx*cosPhi + sinPhi*Lz;
    A[1] = Ly;
    A[2] = -Lx*sinPhi - cosPhi*Lz;

    Lx=((float)compass.m.x-calVal[1][0])/calVal[1][3];
    Ly=((float)compass.m.y-calVal[1][1])/calVal[1][4];
    Lz=((float)compass.m.z-calVal[1][2])/calVal[1][5];
    B[0] = -Lx*cosPhi + sinPhi*Lz;
    B[1] = Ly;
    B[2] = -Lx*sinPhi - cosPhi*Lz;

    Lx=(((float)gyro.g.x)-Gc[0])*GYRO_SENS;
    Ly=(((float)gyro.g.y)-Gc[1])*GYRO_SENS;
    Lz=(((float)gyro.g.z)-Gc[2])*GYRO_SENS;
    G[0] = -Lx*cosPhi + sinPhi*Lz;
    G[1] = Ly;
    G[2] = -Lx*sinPhi - cosPhi*Lz;
    //TODO: insert correction due to off CM positioning of accelerometer 
    
    psAltitude = ps.pressureToAltitudeMeters(pressure);

    sonar_us=sonar_us_vol;
    sonar.ping_timer(echoCheck);

////////////////////////////CALCULATE ORIENTATION//////////////

    static float v1[2][3];
    static float v2[2][3];
    for(int i=0;i<3;i++)
    {
        v1[0][i]=A[i]*0.5;
        v1[1][i]=B[i]*2.;
        v2[0][i]=Ae[i]*0.5;
        v2[1][i]=Be[i]*2.;
    }
    static float R[3][3];
    float dq[4];
    dq[0]=1.0;
    dq[1]=G[0]*dt*0.5;
    dq[2]=G[1]*dt*0.5;
    dq[3]=G[2]*dt*0.5;
    float m=sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]+dq[3]*dq[3]);
    dq[0]/=m;
    dq[1]/=m;
    dq[2]/=m;
    dq[3]/=m;

    float tq[4];
    //tq=q*dq
    tq[0] = dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3];
    tq[1] = dq[0]*q[1] + q[0]*dq[1] - dq[2]*q[3] + dq[3]*q[2];
    tq[2] = dq[0]*q[2] + q[0]*dq[2] - dq[3]*q[1] + dq[1]*q[3];
    tq[3] = dq[0]*q[3] + q[0]*dq[3] - dq[1]*q[2] + dq[2]*q[1];
    q[0]=tq[0];
    q[1]=tq[1];
    q[2]=tq[2];
    q[3]=tq[3];

    R[0][0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
    R[0][1]=2.*(q[1]*q[2]+q[0]*q[3]);
    R[0][2]=2.*(q[1]*q[3]-q[0]*q[2]);

    R[1][0]=2.*(q[1]*q[2]-q[0]*q[3]);
    R[1][1]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
    R[1][2]=2.*(q[2]*q[3]+q[0]*q[1]);

    R[2][0]=2.*(q[1]*q[3]+q[0]*q[2]);
    R[2][1]=2.*(q[2]*q[3]-q[0]*q[1]);
    R[2][2]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
    if(first>0)
    {
        for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            R[i][j]=0.;
        first--;
    }
    getOrientation(v1,v2,q,R,0.0);
    /////////////////////////////////CALCULATE VELOCITY AND POSITION///////////////
    R[0][0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
    R[0][1]=2.*(q[1]*q[2]+q[0]*q[3]);
    R[0][2]=2.*(q[1]*q[3]-q[0]*q[2]);

    R[1][0]=2.*(q[1]*q[2]-q[0]*q[3]);
    R[1][1]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
    R[1][2]=2.*(q[2]*q[3]+q[0]*q[1]);

    R[2][0]=2.*(q[1]*q[3]+q[0]*q[2]);
    R[2][1]=2.*(q[2]*q[3]-q[0]*q[1]);
    R[2][2]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
    /*
    velocity[2]+=GRAV_ACC*dt;
    for(int i=0;i<3;i++)
    {
        velocity[i]+=(R[i][0]*A[0]+R[i][1]*A[1]+R[i][2]*A[2])*GRAV_ACC*dt; 
        position[i]+=velocity[i]*dt;
    }*/

    /////Kalman filter to obtain position and velocity (and acceleration)
    float Phi[3][3]={{1., dt, dt*dt*.5},   {0.,1.0,dt},   {0.0,0.0,1.0}};
    float x_[3];

    //State estimate extrapolation
    x_[0]=x[0] + x[1]*dt + x[2]*.5*dt*dt;
    x_[1]=x[1] + x[2]*dt;
    x_[2]=x[2];
    
    //Error covariance extrapolation
    for(int j=0;j<3;j++)
    for(int k=0;k<3;k++)
    {
        float s=0.;
        for(int l=j;l<3;l++)
        for(int m=k;m<3;m++) //diagonal Phi => start at j,k
            s+=Phi[j][l] * P_posteriori[l][m] * Phi[k][m];
        P_priori[j][k]=s;
    }
    float accQ=1e3;
    P_priori[2][2]+=dt*accQ;

    P_priori[2][1]+=accQ*dt*dt*.5;
    P_priori[1][2]+=accQ*dt*dt*.5;
    
    P_priori[0][2]+=accQ*dt*dt*dt/6.;
    P_priori[1][1]+=accQ*dt*dt*dt/3.;
    P_priori[2][0]+=accQ*dt*dt*dt/6.;

    P_priori[0][1]+=accQ*dt*dt*dt*dt/8.;
    P_priori[1][0]+=accQ*dt*dt*dt*dt/8.;

    P_priori[0][0]+=accQ*dt*dt*dt*dt*dt/20.;

    //Kalman gain
    float S11=P_priori[0][0]+(sonar_us==0?50.*50.:0.04*0.04);
    float S12=P_priori[0][2];
    float S22=P_priori[2][2]+0.2*0.2;
    float detS=S11*S22-S12*S12;
    float iS11=S22/detS;
    float iS22=S11/detS;
    float iS12=-S12/detS;
    float K11=P_priori[0][0]*iS11 + P_priori[0][2]*iS12;
    float K12=P_priori[0][0]*iS12 + P_priori[0][2]*iS22;
    float K21=P_priori[0][1]*iS11 + P_priori[1][2]*iS12;
    float K22=P_priori[0][1]*iS12 + P_priori[1][2]*iS22;
    float K31=P_priori[0][2]*iS11 + P_priori[2][2]*iS12;
    float K32=P_priori[0][2]*iS12 + P_priori[2][2]*iS22;
    
    //Error covariance update
    float M[3][3]={{1.-K11,0.,-K12}, {-K21,1.,-K22}, {-K31,.0,1.-K32}};
    for(int j=0;j<3;j++)
    for(int k=0;k<3;k++)
    {
        float s=0.;
        for(int l=0;l<3;l++)
            s+=M[j][l] * P_priori[l][k];
        P_posteriori[j][k]=s;
    }

    //State estimate observational update
    float ze[2]={sonar_us*.5*SOUND_SPEED*1e-6 - x_[0],
                 ( -1.-(R[0][2]*A[0]+R[1][2]*A[1]+R[2][2]*A[2]) )*GRAV_ACC - x_[2]};

    x[0]=x_[0] + K11*ze[0] + K12*ze[1];
    x[1]=x_[1] + K21*ze[0] + K22*ze[1];
    x[2]=x_[2] + K31*ze[0] + K32*ze[1];
    ////////////////////////////////////OUTPUT SERIAL DATA/////////////////////////
#ifdef COMMUNICATE
      Serial.print("start,");
      for(int i=0;i<3;i++)
      {
        Serial.print(A[i]);
        Serial.print(",");
      }
      for(int i=0;i<3;i++)
      {
        Serial.print(B[i]);
        Serial.print(",");
      }

      Serial.print(psAltitude);
      Serial.print(",");
      Serial.print(psTemperature);
      Serial.print(",");

      Serial.print(G[0]);
      Serial.print(",");
      Serial.print(G[1]);
      Serial.print(",");
      Serial.println(G[2]);
      
    Serial.print("orient,");
    for(int i=0;i<4;i++)
    {
        Serial.print(q[i]*100.);
        Serial.print(",");
    }
    Serial.println("");

    //Serial.print("analog,");
    //Serial.print(analogRead(RIGHT_PHOTO));
    //Serial.println("");
    Serial.print("ping,");
    //Serial.print(1e2*sqrt(P_posteriori[0][0]));
    Serial.print(x[0]*100.);
    Serial.print(",");
    Serial.print(P_posteriori[0][0]*100.);
    Serial.println("");
#endif
    ///////////////////////////////CALCULATE PHYSICAL OUTPUTS////////////////

    float qwni=1./sqrt(q[0]*q[0]+q[3]*q[3]);
    qw[0]=q[0]*qwni*cosHalfLandAngle;
    qw[1]=-q[3]*qwni*sinHalfLandAngle;
    qw[2]=q[0]*qwni*sinHalfLandAngle;
    qw[3]=q[3]*qwni*cosHalfLandAngle;
    

    float e[4];
    //e=q^\bar qw
    e[0] = q[0]*qw[0] + q[1]*qw[1] + q[2]*qw[2] + q[3]*qw[3];
    e[1] = q[0]*qw[1] - q[1]*qw[0] - q[2]*qw[3] + q[3]*qw[2];
    e[2] = q[0]*qw[2] + q[1]*qw[3] - q[2]*qw[0] - q[3]*qw[1];
    e[3] = q[0]*qw[3] - q[1]*qw[2] + q[2]*qw[1] - q[3]*qw[0];

    //natural log of e, generator of rotation
    float lne[4];
    float sqrv=e[1]*e[1]+e[2]*e[2]+e[3]*e[3];
    float acos_a=asin(sqrv)*(e[0]>0.?1.:-1.); //more stable way to calculate acos(a) since v^2+a^2=1
    float normv=sqrt(sqrv);
    lne[0]=0.;
    lne[1]=e[1]/normv*acos_a;
    lne[2]=e[2]/normv*acos_a;
    lne[3]=e[3]/normv*acos_a;

    /*Serial.print(lne[1]);
    Serial.print(", ");
    Serial.print(lne[2]);
    Serial.print(", ");
    Serial.println(lne[3]);*/
   /* us[t%3]=sonar.ping();
    t2=0;
    while(us[t%3]==0 && t2<-4)
    {
        t2++;    
        us[t%3]=sonar.ping();
    }
    if(t%3==2)
    {
        unsigned int gus;
        if(us[0]<us[1])
            if(us[0]<us[2])
                if(us[1]<us[2])
                    gus=us[1];
                else
                    gus=us[2];
            else
                gus=us[0];
        else
            if(us[1]<us[2])
                if(us[0]<us[2])
                    gus=us[0];
                else
                    gus=us[2];
            else
                gus=us[1];
        //if(gus==0)
        //    position[2]=MAX_SONAR_DIST;//TODO update ground pressure here
        //else
        //    position[2]=gus*.5*SOUND_SPEED*1e-6;//TODO use pressure

    }*/
    i_roll= constrain( i_roll + lne[1]*dt, -integrateMax, integrateMax );
    i_pitch= constrain( i_pitch + lne[2]*dt, -integrateMax, integrateMax );
    u_roll = 0.;//constrain( - lne[1]*P - i_roll*I + G[0]*D, -uMax , uMax);
    u_pitch = 0.;//-uMax+2.*uMax*constrain(x[0]/MAX_SONAR_DIST,0.,1.);  //constrain( - lne[2]*P - i_pitch*I + G[1]*D, -uMax, uMax);
    if(us==0) u_pitch=uMax;
    u_skew=0.;
    for(int i=0;i<4;i++)
    {
        //servos[i].writeMicroseconds( (int)(1500.+constrain(u_roll*roll[i]+u_pitch*pitch[i],-1.,1.)*maxCSus) );
    }
    ////////////////////////////OUTPUT SIGNALS////////////////////
    digitalWrite(GREEN_LED, (t%100==0)!=(sonar_us==0)?LOW:HIGH);
    t++;
}
