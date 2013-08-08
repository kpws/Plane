void blink()
{
    for(int i=0;i<10;i++)
    {
        digitalWrite(GREEN_LED,LOW);
        delay(100);
        digitalWrite(GREEN_LED,HIGH);
        delay(100);
    }
}

/*void calG()
  {
  blink();
  for(int i=0;i<3;i++)
  Gss[i]=0;
  for(int i=0;i<200;i++)
  {
  digitalWrite(led,HIGH);
  gyro.read();
  unsigned long mi=micros();
  float dt=(mi-lmi)*1e-6;
  lmi=mi;

  compass.read();
  digitalWrite(led,LOW);
  A[0]=((float)compass.a.x-calVal[0][0])/calVal[0][3];
  A[1]=((float)compass.a.y-calVal[0][1])/calVal[0][4];
  A[2]=((float)compass.a.z-calVal[0][2])/calVal[0][5];
  B[0]=((float)compass.m.x-calVal[1][0])/calVal[1][3];
  B[1]=((float)compass.m.y-calVal[1][1])/calVal[1][4];
  B[2]=((float)compass.m.z-calVal[1][2])/calVal[1][5];
  G[0]=gyro.g.x;
  G[1]=gyro.g.y;
  G[2]=gyro.g.z;
  float v1[2][3];
  float v2[2][3];
  for(int j=0;j<3;j++)
  {
  v1[0][j]=Ae[j];
  v1[1][j]=Be[j];
  v2[0][j]=A[j];
  v2[1][j]=B[j];
  }
  float qi[4];
  qi[0]=q[0];
  qi[1]=-q[1];
  qi[2]=-q[2];
  qi[3]=-q[3];
  float R[3][3];
  for(int j=0;j<3;j++)
  for(int k=0;k<3;k++)
  R[j][k]=0.;
  getOrientation(v1,v2,2.,q,R,0.5);
  float dq[4];
  dq[0] = qi[0]*q[0] - qi[1]*q[1] - qi[2]*q[2] - qi[3]*q[3];
  dq[1] = qi[0]*q[1] + q[0]*qi[1] - qi[2]*q[3] + qi[3]*q[2];
  dq[2] = qi[0]*q[2] + q[0]*qi[2] - qi[3]*q[1] + qi[1]*q[3];
  dq[3] = qi[0]*q[3] + q[0]*qi[3] - qi[1]*q[2] + qi[2]*q[1];
  float m=sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]+dq[3]*dq[3]);
  for(int j=0;j<3;j++)
  dq[j]/=m;
//float f = 2. * acos(dq[0])/sqrt(1.-dq[0]*dq[0]);
m=sqrt(dq[1]*dq[1]+dq[2]*dq[2]+dq[3]*dq[3]);
float f = -2. * atan2(m,dq[0])/m;
for(int j=0;j<3;j++)
{
float v=(G[j]-Gc[j]);
float av=fabs(v);
*float s=(dq[1+j]*f)/v;
if(Gss[j]==0.)
Gs[j]=s;
else
Gs[j]+=(s-Gs[j])*av/(Gss[j]+av);*
if(av>5000.)
{
Gs[j]+=fabs(dq[1+j]*f);
Gss[j]+=av*dt;
}
//Serial.print(dq[1+j]*f);
//Serial.print(",");
}
#ifdef COMMUNICATE 
Serial.print(Gs[0]/Gss[0]*1e5);
Serial.print("     ");
Serial.print(Gs[1]/Gss[1]*1e5);
Serial.print("     ");
Serial.print(Gs[2]/Gss[2]*1e5);
Serial.println("");
#endif
delay(100);
}
for(int i=0;i<3;i++)
Gs[i]/=Gss[i];
}*/

void initCal()
{
    calVal[0][0]=-39;
    calVal[0][1]=3;
    calVal[0][2]=13;
    calVal[0][3]=1037;
    calVal[0][4]=1034;
    calVal[0][5]=1025;

    calVal[1][0]=82;
    calVal[1][1]=-137;
    calVal[1][2]=15;
    calVal[1][3]=634;
    calVal[1][4]=648;
    calVal[1][5]=538;

    calPoints=6;
    calPoint[0][0]=-312.30;
    calPoint[0][1]=113.75;
    calPoint[0][2]=997.30;
    calPoint[0][3]= 269.80;
    calPoint[0][4]= -99.35;
    calPoint[0][5]= -507.70;

    calPoint[1][0]=773.60;
    calPoint[1][1]=-135.10;
    calPoint[1][2]=632.45;
    calPoint[1][3]= -54.45;
    calPoint[1][4]= 21.90;
    calPoint[1][5]= 550.55;

    calPoint[2][0]=-123.45;
    calPoint[2][1]=-755.80;
    calPoint[2][2]=706.00;
    calPoint[2][3]= -519.70;
    calPoint[2][4]= 4.70;
    calPoint[2][5]= -102.95;

    calPoint[3][0]=-939.70;
    calPoint[3][1]=-241.00;
    calPoint[3][2]=-439.30;
    calPoint[3][3]= 703.75;
    calPoint[3][4]= 25.80;
    calPoint[3][5]= 132.05;

    calPoint[4][0]=466.50;
    calPoint[4][1]=-769.00;
    calPoint[4][2]=-457.40;
    calPoint[4][3]= 272.05;
    calPoint[4][4]= -729.65;
    calPoint[4][5]= 179.30;

    calPoint[5][0]=-55.20;
    calPoint[5][1]=944.70;
    calPoint[5][2]=-410.15;
    calPoint[5][3]=216.90;
    calPoint[5][4]=495.70;
    calPoint[5][5]=143.85;


    Gc[0]=0.0;
    Gc[1]=0.0;
    Gc[2]=0.0;

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
void cal()
{
    if(calPoints<MAX_CP)
    {
        blink();
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
            pressure = ps.readPressureMillibars();
            values[i][9]=ps.pressureToAltitudeMeters(pressure);

            //for(int j=0;j<N_SUN_SENSORS;j++)
            //    values[i][10+j]=analogRead(A0+j);

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
        float margin=0.01;
        if(  vars[0]+vars[1]+vars[2]>
                (means[0]*means[0]+means[1]*means[1]+means[2]*means[2])*margin*margin  )
            ok=0;
        if(  vars[3]+vars[4]+vars[5]>
                (means[3]*means[3]+means[4]*means[4]+means[5]*means[5])*margin*margin )
            ok=0;
        if(vars[9]>2.*2.)
            ok=0;

#ifdef COMMUNICATE
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
        Serial.print("calG,");
        Serial.print(means[6]);
        Serial.print(",");
        Serial.print(means[7]);
        Serial.print(",");
        Serial.println(means[8]);
#endif

        if(ok)
        {
            for(int i=0;i<10;i++)
                calPoint[calPoints][i]=means[i];
            calPoints++;
            for(int i=0;i<20;i++)
            {
                digitalWrite(GREEN_LED,LOW);
                delay(20);
                digitalWrite(GREEN_LED,HIGH);
                delay(20);
            }
            /*for(int j=0;j<3;j++)
            {
                Gc[j]=.0;
                for(int k=0;k<calPoints;k++)
                    Gc[j]+=calPoint[k][6+j];
                Gc[j]/=calPoints;
            }*/

            if(calPoints>=6)
            {
                float grad[6],**hess;
                hess=malloc(sizeof(float*)*6);
                for(int j=0;j<6;j++)
                    hess[j]=malloc(sizeof(float)*6);
                for(int am=0;am<2;am++)
                {
                    float x[6];
                    for(int j=0;j<6;j++)
                        x[j]=calVal[am][j];
                    int converged=0;
                    int offset=am*3;
                    while(!converged)
                    {
                        for(int j=0;j<6;j++)
                        {
                            float h=.1;
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
#ifdef COMMUNICATE
                        Serial.print("iter,");
                        Serial.print(x[0]);
                        Serial.print(",");
                        Serial.print(x[3]);
                        Serial.println("");
#endif
                    }
                    for(int j=0;j<6;j++)
                        calVal[am][j]=x[j];
                }          
                for(int j=0;j<6;j++)
                    free(hess[j]);
                free(hess);
#ifdef COMMUNICATE
                Serial.print("calRes A,");
                for(int j=0;j<6;j++)
                {
                    Serial.print(calVal[0][j]);
                    Serial.print(",");
                }
                Serial.println("");
                Serial.print("calRes M,");
                for(int j=0;j<6;j++)
                {
                    Serial.print(calVal[1][j]);
                    Serial.print(",");
                }
                Serial.println("");
#endif
            }
        }
        digitalWrite(GREEN_LED,LOW);
    }
}
