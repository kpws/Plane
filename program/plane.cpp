#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LPS331.h>
#include <LSM303.h>
#include <L3G.h>

#define COMMUNICATE

#include "qr_fact.c"
#include "orientation.c"
//#include "svdcmp.c"
//#include "magfield.c"

const int WARM_UP=0;
const int CALIBRATE=1;
const int READY=3;
const int MAX_CP=16;
const int CAL_MEANS=20;
//const int N_SUN_SENSORS=8;
const float GYRO_SENS=17.5e-3*PI/180.;
const int GREEN_LED = 13;
/*
   const int lEngine=0;
   const int rEngine=1;
   const int lAileron=2;
   const int rAileron=3;
   const int lElev=4;
   const int rElev=5;
   const int N_SERVOS=6;
   */
//Servo servos[N_SERVOS];

LPS331 ps;
LSM303 compass;
L3G gyro;

int mode;
long t;
unsigned long lmi;
float dt;

float calPoint[MAX_CP][10];
int calPoints=0;
float calVal[2][6];
float Gc[3];

float longitude,latitude,altitude;
float Be[3],Ae[3];//,Se[3];

float B[3],A[3],G[3];//,S[3];
//int sunSens[N_SUN_SENSORS];
float pressure,psAltitude,psTemperature;

float q[4];

#include "calibrate.cpp"
//#include "sensors.c"

void setup()
{
    pinMode(GREEN_LED, OUTPUT);
#ifdef COMMUNICATE
    Serial.begin(115200);
#endif
    for(float i=80;i<1500.;i*=1.1)
    {

        tone(8,i,30);
        delay(30);
        tone(8,i*2,30);
        delay(30);
    }

    //for(int i=0;i<N_SERVOS;i++)
    //    servos[i].attach(8);

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
    Be[0]=15740.9;
    Be[1]=1179.2; //nT, Linkoping, 2013.4, IGRF11
    Be[2]=-48455.2;
    float Bem=sqrt(Be[0]*Be[0]+Be[1]*Be[1]+Be[2]*Be[2]);
    for(int i=0;i<3;i++)
        Be[i]/=Bem; 

    Ae[0]=0.;
    Ae[1]=0.;
    Ae[2]=1.;

    q[0]=1.;
    q[1]=0.;
    q[2]=0.;
    q[3]=0.;

    lmi=0;

    mode=CALIBRATE;
    initCal();

    t=0;
    dt=0;
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

    A[0]=((float)compass.a.x-calVal[0][0])/calVal[0][3];
    A[1]=((float)compass.a.y-calVal[0][1])/calVal[0][4];
    A[2]=((float)compass.a.z-calVal[0][2])/calVal[0][5];

    B[0]=((float)compass.m.x-calVal[1][0])/calVal[1][3];
    B[1]=((float)compass.m.y-calVal[1][1])/calVal[1][4];
    B[2]=((float)compass.m.z-calVal[1][2])/calVal[1][5];

    G[0]=(((float)gyro.g.x)-Gc[0])*GYRO_SENS;
    G[1]=(((float)gyro.g.y)-Gc[1])*GYRO_SENS;
    G[2]=(((float)gyro.g.z)-Gc[2])*GYRO_SENS;

    psAltitude = ps.pressureToAltitudeMeters(pressure);

////////////////////////////CALCULATE ORIENTATION//////////////

    static float v1[2][3];
    static float v2[2][3];
    for(int i=0;i<3;i++)
    {
        v1[0][i]=Ae[i]*0.5;
        v1[1][i]=Be[i]*2.;
        v2[0][i]=A[i]*0.5;
        v2[1][i]=B[i]*2.;
    }
    static float R[3][3];
    float dq[4];
    dq[0]=1.0;
    dq[1]=-G[0]*dt*0.5;
    dq[2]=-G[1]*dt*0.5;
    dq[3]=-G[2]*dt*0.5;
    float m=sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]+dq[3]*dq[3]);
    dq[0]/=m;
    dq[1]/=m;
    dq[2]/=m;
    dq[3]/=m;

    float tq[4];
    tq[0] = dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3];
    tq[1] = dq[0]*q[1] + q[0]*dq[1] + dq[2]*q[3] - dq[3]*q[2];
    tq[2] = dq[0]*q[2] + q[0]*dq[2] + dq[3]*q[1] - dq[1]*q[3];
    tq[3] = dq[0]*q[3] + q[0]*dq[3] + dq[1]*q[2] - dq[2]*q[1];
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

    getOrientation(v1,v2,q,R,0.0);
    
    ////////////////////////////////////OUTPUT SERIAL DATA/////////////////////////
#ifdef COMMUNICATE
   /* Serial.print("start,");
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
     */ 
    Serial.print("orient,");
    for(int i=0;i<4;i++)
    {
        Serial.print(q[i]*100.);
        Serial.print(",");
    }
    Serial.println("");
#endif
    ///////////////////////////////CALCULATE PHYSICAL OUTPUTS////////////////

    ////////////////////////////OUTPUT SIGNALS////////////////////
    digitalWrite(GREEN_LED, t%100?LOW:HIGH);
    t++;
}
