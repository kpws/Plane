#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LPS331.h>
#include <LSM303.h>
#include <L3G.h>

#include "qr_fact.c"

const float Pi = 3.141593; 
const int WARM_UP=0;
const int CALIBRATE=1;
const int READY=2;
const int MAX_CP=16;
const int CAL_MEANS=20;

const int led = 13;
Servo s1;
Servo s2;
float t=0;

LPS331 ps;
LSM303 compass;
L3G gyro;
int mode;

float calPoint[MAX_CP][10];
int calPoints=0;
float calVal[2][6];
float x[6]={0,0,0,1000,1000,1000};

void setup()
{
    pinMode(led, OUTPUT);
    Serial.begin(115200);
    //	s1.attach(8);
    //	s2.attach(9);

    Wire.begin();

    ps.init();
    ps.enableDefault();
    //ps.writeReg(LPS331_RES_CONF, 0b11111010);

    compass.init();
    compass.enableDefault(); 

    gyro.init();
    gyro.enableDefault();

    mode=CALIBRATE;
    calVal[0][0]=0;
    calVal[0][1]=0;
    calVal[0][2]=0;
    calVal[0][3]=1000;
    calVal[0][4]=1000;
    calVal[0][5]=1000;
    
    calVal[1][0]=0;
    calVal[1][1]=0;
    calVal[1][2]=0;
    calVal[1][3]=600;
    calVal[1][4]=600;
    calVal[1][5]=600;
    
    calPoints=0;
    calPoint[0][0]=-312.30;
    calPoint[0][1]=113.75;
    calPoint[0][2]=997.30;

    calPoint[1][0]=773.60;
    calPoint[1][1]=-135.10;
    calPoint[1][2]=632.45;
    
    calPoint[2][0]=-123.45;
    calPoint[2][1]=-755.80;
    calPoint[2][2]=706.00;
    
    calPoint[3][0]=-939.70;
    calPoint[3][1]=-241.00;
    calPoint[3][2]=-439.30;
    
    calPoint[4][0]=466.50;
    calPoint[4][1]=-769.00;
    calPoint[4][2]=-457.40;
    
    calPoint[5][0]=-55.20;
    calPoint[5][1]=944.70;
    calPoint[5][2]=-410.15;
}
double ellipseError(double xc,double yc,double zc,double rx,double ry,double rz,int offset,int N)
{
    double error=0.;
    for(int i=0;i<N;i++)
    {
        float e=(calPoint[i][offset+0]-xc)*(calPoint[i][offset+0]-xc)/(rx*rx) +
            (calPoint[i][offset+1]-yc)*(calPoint[i][offset+1]-yc)/(ry*ry) + 
            (calPoint[i][offset+2]-zc)*(calPoint[i][offset+2]-zc)/(rz*rz) -1.; 
        error+=e*e;
    }
    return error;
}
void loop()
{
    const int bufLen=32;
    char buf[bufLen];
    if(Serial.available())
    {
        if(Serial.readBytesUntil('\n', buf, bufLen))
        {
            if(strncmp(buf,"ledOn",5)==0)
            {
                digitalWrite(led, HIGH);
            }
            if(strncmp(buf,"ledOff",6)==0)
            {
                digitalWrite(led, LOW);
            }
            if(strncmp(buf,"cal",3)==0)
            {
                if(mode==CALIBRATE and calPoints<MAX_CP)
                {
                    for(int i=0;i<10;i++)
                    {
                        digitalWrite(led,LOW);
                        delay(100);
                        digitalWrite(led,HIGH);
                        delay(100);
                    }
                    int values[CAL_MEANS][10];
                    for(int i=0;i<CAL_MEANS;i++)
                    {
                        compass.read();
                        values[i][0]=compass.a.x;	
                        values[i][1]=compass.a.y;	
                        values[i][2]=compass.a.z;	
                        values[i][3]=compass.m.x;	
                        values[i][4]=compass.m.y;	
                        values[i][5]=compass.m.z;	
                        gyro.read();
                        values[i][6]=gyro.g.x;	
                        values[i][7]=gyro.g.y;	
                        values[i][8]=gyro.g.z;	
                        float pressure = ps.readPressureMillibars();
                        values[i][9]=ps.pressureToAltitudeMeters(pressure);
                        delay(100);
                    }
                    float means[10];
                    float vars[10];
                    for(int i=0;i<10;i++)
                    {
                        means[i]=0.0;
                        for(int j=0;j<CAL_MEANS;j++)
                            means[i]+=values[j][i];
                        means[i]/=CAL_MEANS;
                        vars[i]=0.0;
                        for(int j=0;j<CAL_MEANS;j++)
                            vars[i]+=(values[j][i]-means[i])*(values[j][i]-means[i]);
                        vars[i]/=(CAL_MEANS-1);
                    }
                    int ok=1;
                    float margin=0.05;
                    if(  vars[0]+vars[1]+vars[2]>
                            (means[0]*means[0]+means[1]*means[1]+means[2]*means[2])*margin*margin  )
                        ok=0;
                    if(  vars[3]+vars[4]+vars[5]>
                            (means[3]*means[3]+means[4]*means[4]+means[5]*means[5])*margin*margin )
                        ok=0;
                    if(vars[9]>2.*2.)
                        ok=0;
                    Serial.print("calA,");
                    Serial.print(means[0]);
                    Serial.print(",");
                    Serial.print(means[1]);
                    Serial.print(",");
                    Serial.print(means[2]);
                    Serial.print(",");
                    Serial.println(sqrt(vars[0]+vars[1]+vars[2]));
                    Serial.print("calM,");
                    Serial.print(means[3]);
                    Serial.print(",");
                    Serial.print(means[4]);
                    Serial.print(",");
                    Serial.print(means[5]);
                    Serial.print(",");
                    Serial.println(sqrt(vars[3]+vars[4]+vars[5]));

                    if(ok)
                    {
                        for(int i=0;i<10;i++)
                            calPoint[calPoints][i]=means[i];
                        calPoints++;
                        for(int i=0;i<20;i++)
                        {
                            digitalWrite(led,LOW);
                            delay(20);
                            digitalWrite(led,HIGH);
                            delay(20);
                        }
                        if(calPoints>=6)
                        {
                            float grad[6],**hess;
                            hess=malloc(sizeof(float*)*6);
                            float h=1.;
                            for(int j=0;j<6;j++)
                                hess[j]=malloc(sizeof(float)*6);
                            for(int am=0;am<2;am++)
                            {
                                for(int j=0;j<6;j++)
                                    x[j]=calVal[am][j];
                                int converged=0;
                                int offset=am*3;
                                while(!converged)
                                {
                                    Serial.print("iter,");
                                    for(int j=0;j<6;j++)
                                    {
                                        float old=x[j];
                                        x[j]=old-h;
                                        float left=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                        x[j]=old+h;
                                        float right=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                        x[j]=old;
                                        grad[j]=(right-left)/(2.*h);
                                        for(int k=0;k<6;k++)
                                        {
                                            float old1=x[j];
                                            float old2=x[k];
                                            x[j]-=h;
                                            x[k]-=h;
                                            float p1=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                            x[j]=old1;
                                            x[k]=old2;
                                            x[j]-=h;
                                            x[k]+=h;
                                            float p2=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                            x[j]=old1;
                                            x[k]=old2;
                                            x[j]+=h;
                                            x[k]-=h;
                                            float p3=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                            x[j]=old1;
                                            x[k]=old2;
                                            x[j]+=h;
                                            x[k]+=h;
                                            float p4=ellipseError(x[0],x[1],x[2],x[3],x[4],x[5],offset,calPoints);
                                            hess[j][k]=(p4-p2-p3+p1)/(4.*h*h);
                                            x[j]=old1;
                                            x[k]=old2;
                                        }
                                    }
                                    float step[6];
                                    qr_least_squares(hess, grad, step,6,6);
                                    converged=1;
                                    for(int j=0;j<6;j++)
                                    {
                                        if(fabs(step[j])>0.005)
                                            converged=0;
                                        x[j]-=step[j];
                                    }
                                    Serial.print(x[0]);
                                    Serial.print(",");
                                    Serial.print(x[3]);
                                    Serial.print(",");
                                }
                                for(int j=0;j<6;j++)
                                    calVal[am][j]=x[j];
                            }          
                            for(int j=0;j<6;j++)
                                free(hess[j]);
                            free(hess);
                            Serial.println("");
                            Serial.print("calRes,");
                            Serial.print(x[0]);
                            Serial.print(",");
                            Serial.print(x[1]);
                            Serial.print(",");
                            Serial.print(x[2]);
                            Serial.print(",");
                            Serial.println(x[3]);
                        }
                    }
                    digitalWrite(led,LOW);
                }
            }
        }
    }
    /*	for(int i=100;i>0;i-=1)
        {
        digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(i);               // wait for a second
        digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
        delay(i);               // wait for a second
        Serial.println(i);
        }
        */
    int val0 = 0;
    int val1 = 0;
    const int N=1;
    for(int i=0;i<N;i++)
    {
        val0+=analogRead(A0); // read analog input pin 0
        val1+=analogRead(A1); // read analog input pin 0
        //delay(1);
    }
    //		digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    //		delay(2000/(val+10));               // wait for a second
    //		digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    //		delay(2000/(val+10));               // wait for a second

    //Serial.print(val0); // prints the value read*/
    //Serial.print(", "); // prints the value read*/
    //Serial.println(val1); // prints the value read*/
    /*delay(5); // wait 10ms for next reading
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(5); // wait 10ms for next reading
      digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)*/
    /*angle+=(float(val)*360./1024.+30.-angle)*0.1;
      t=t+0.02;//+val*0.002;
      if(t>2.*Pi) t-=2.*Pi;
      float span=300.;
    //s1.write(90.+70.*sin(t));
    //s2.write(90.+70.*cos(t));
    //delay(20)*/;

    compass.read();

    Serial.print("start,");
    Serial.print(((float)compass.a.x-calVal[0][0])/calVal[0][3]);
    Serial.print(",");
    Serial.print(((float)compass.a.y-calVal[0][1])/calVal[0][4]);
    Serial.print(",");
    Serial.print(((float)compass.a.z-calVal[0][2])/calVal[0][5]);
    Serial.print(",");
    Serial.print(((float)compass.m.x-calVal[1][0])/calVal[1][3]);
    Serial.print(",");
    Serial.print(((float)compass.m.y-calVal[1][1])/calVal[1][4]);
    Serial.print(",");
    Serial.print(((float)compass.m.z-calVal[1][2])/calVal[1][5]);
    Serial.print(",");


    float pressure = ps.readPressureMillibars();
    float altitude = 0;
    int N2=1;
    for(int i=0;i<N2;i++)
    {
        altitude+=ps.pressureToAltitudeMeters(pressure);
    }
    altitude/=N2;
    float temperature = ps.readTemperatureC();

    Serial.print(altitude);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    //Serial.print(pressure);
    //Serial.print(" mbar\ta: ");
    //Serial.print(altitude);
    //Serial.println(" m\tt: ");
    //Serial.print(temperature);
    //Serial.println(" deg C");

    gyro.read();

    Serial.print((int)gyro.g.x);
    Serial.print(",");
    Serial.print((int)gyro.g.y);
    Serial.print(",");
    Serial.println((int)gyro.g.z);
}
