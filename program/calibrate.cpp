
void initCal()
{
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
void cal()
{
    if(calPoints<MAX_CP)
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
