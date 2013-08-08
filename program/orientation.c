//float **F;

void getOrientation(float v1[2][3],float v2[2][3],float q[4],float R[3][3],float h)
{
    for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
    {
        for(int k=0;k<2;k++)
            R[i][j]+=v1[k][i]*v2[k][j];
    }
    float F[4][4];
    F[0][0]=R[0][0]+R[1][1]+R[2][2]+h;
    F[0][1]=R[1][2]-R[2][1];
    F[0][2]=R[2][0]-R[0][2];
    F[0][3]=R[0][1]-R[1][0];

    F[1][1]=R[0][0]-R[1][1]-R[2][2]+h;
    F[1][2]=R[0][1]+R[1][0];
    F[1][3]=R[0][2]+R[2][0];
    
    F[2][2]=-R[0][0]+R[1][1]-R[2][2]+h;
    F[2][3]=R[1][2]+R[2][1];

    F[3][3]=-R[0][0]-R[1][1]+R[2][2]+h;

    for(int i=1;i<4;i++)
        for(int j=0;j<i;j++)
            F[i][j]=F[j][i];

    int converged=0;
    int its=0;
    while(!converged && its<100)
    {
        float q2[4];
        float m=0;
        for(int i=0;i<4;i++)
        {
            q2[i]=0.;
            for(int j=0;j<4;j++)
                q2[i]+=F[i][j]*q[j];
        }
        float q3[4];
        for(int i=0;i<4;i++)
        {
            q3[i]=0.;
            for(int j=0;j<4;j++)
                q3[i]+=F[i][j]*q2[j];
            m+=q3[i]*q3[i];
        }
        if(m==0.)
        {
            q3[0]=1.;
            m=1.;
        }
        m=sqrt(m);
        converged=1;
        for(int i=0;i<4;i++)
        {
            float n=q3[i]/m;
            if(fabs(q[i]-n)>0.005)
                converged=0;
            q[i]=n;
        }
        its++;
    }
}
