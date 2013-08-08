void readSensors()
{
   // for(int i=0;i<N_SUN_SENSORS;i++)
    //    sunSens[i]=analogRead(A0+i);

    compass.read();
    gyro.read();

    pressure = ps.readPressureMillibars();
    psTemperature = ps.readTemperatureC();

}
void processSensors()
{
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
}
