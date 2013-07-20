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
int cycles=0;


int calPoint[MAX_CP][10];
int calPoints=0;

void setup()
{
	pinMode(led, OUTPUT);
	//Serial.begin(9600);
	Serial.begin(115200);
	//	s1.attach(8);
	//	s2.attach(9);
	//

	Wire.begin();

	ps.init();
	ps.enableDefault();
	//ps.writeReg(LPS331_RES_CONF, 0b11111010);

	compass.init();
	compass.enableDefault(); 

	gyro.init();
	gyro.enableDefault();

	mode=CALIBRATE;
	calPoints=0;
}
double ellipseError(xc,yc,zc,rx,ry,rz,xp,yp,zp,N)
{
    double error=0.;
    for(int i=0;i<N;i++)
    {
        float e=(xp[i]-xc)*(xp[i]-xc)/(rx*rx) + (yp[i]-yc)*(yp[i]-yc)/(ry*ry) + (zp[i]-zc)*(zp[i]-zc)/(rz*rz) - 1.;
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
					if(  vars[0]*vars[0]+vars[1]*vars[1]+vars[2]*vars[2]>
							(means[0]*means[0]+means[1]*means[1]+means[2]*means[2])*margin*margin  )
						ok=0;
					if(  vars[3]*vars[3]+vars[4]*vars[4]+vars[5]*vars[5]>
							(means[3]*means[3]+means[4]*means[4]+means[5]*means[5])*margin*margin )
						ok=0;
					if(vars[9]>2.*2.)
						ok=0;
		            Serial.print("cal,");
		            Serial.print(means[0]);
		            Serial.print(",");
		            Serial.print(means[1]);
		            Serial.print(",");
		            Serial.print(means[2]);
		            //Serial.print(",");
                    //Serial.print(sqrt(means[0]*means[0]+means[1]*means[1]+means[2]*means[2]));
		            Serial.print(",");
		            Serial.println(sqrt(vars[0]*vars[0]+vars[1]*vars[1]+vars[2]*vars[2]));
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
					       /*double **mat=malloc(sizeof(double *)*calPoints); 	
                           double *b=malloc(sizeof(double)*calPoints);
                           for(int i=0;i<calPoints;i++)
                           {
                               mat[i]=malloc(sizeof(double)*7);
                               *mat[i][0]=2.*calPoint[i][0];
                               mat[i][1]=2.*calPoint[i][1];
                               mat[i][2]=2.*calPoint[i][2];
                               mat[i][3]=1.;
                               b[i]=calPoint[i][0]*calPoint[i][0]+
                                   calPoint[i][1]*calPoint[i][1]+calPoint[i][2]*calPoint[i][2];*
                               mat[i][0]=calPoint[i][0]*calPoint[i][0];
                               mat[i][1]=calPoint[i][1]*calPoint[i][1];
                               mat[i][2]=calPoint[i][2]*calPoint[i][2];
                               mat[i][3]=-2.*calPoint[i][0];
                               mat[i][4]=-2.*calPoint[i][1];
                               mat[i][5]=-2.*calPoint[i][2];
                               mat[i][6]=1.;
                               b[i]=1.;
                           }
                           double x[7];
                           qr_least_squares(mat, b, x, calPoints, 7);
                           for(int i=0;i<calPoints;i++)
                               free(mat[i]);
                           free(mat);
                           free(b);*/
                           float x[6]={0,0,0,1000,1000,1000};
                           for(int i=0;i<5;i++)
                           {
                               float e=ellipseError(x[0],x[1],x[2],x[3],x[4])
                           }          
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
		Serial.print((int)compass.m.x);
		Serial.print(",");
		Serial.print((int)compass.m.y);
		Serial.print(",");
		Serial.print((int)compass.m.z);
		Serial.print(",");
		Serial.print((int)compass.a.x);
		Serial.print(",");
		Serial.print((int)compass.a.y);
		Serial.print(",");
		Serial.print((int)compass.a.z);
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
