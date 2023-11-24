#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp> 
using namespace std;
using namespace cv;
double obs[14]={1,1.2,3,2.5,4,4.5,3,6,6,2,5.5,5.5,8,8.5};
double sum(double *p1,int n);
class APF
{
public:
    double Attraction_K;
    double Repulsion_K;
    double Obstacles_dis;
    double a;
    double step;
    double *start_point;
    double *goal_point;
    double *obstacles;
    void APF_init(double Attraction_K,double Repulsion_K,double Obstacles_dis, double a,double step,double *start_point,double *goal_point,double *obstacles)
    {
        this->Attraction_K=Attraction_K;
        this->Repulsion_K=Repulsion_K;
        this->Obstacles_dis=Obstacles_dis;
        this->a=a;
        this->step=step;
        this->start_point=start_point;
        this->goal_point=goal_point;
        this->obstacles=obstacles;
    }
    double * compute_angle(double *start_point,int n);
    double *compute_attraction(double *start_point,double *att_angle);
    double *compute_repulsion(double *start_point,double *angle,int n);
};
double * APF::compute_angle(double *start_point,int n)
{
//     static double Y[8];
    double *Y=new double[n+1];
    double deltax,deltay,r;
    for(int i=0;i<n+1;i++)
    {
        if(i!=0)
        {
            deltax=this->obstacles[(i-1)*2]-start_point[0];
            deltay=this->obstacles[(i-1)*2+1]-start_point[1];
        }
        else
        {
            deltax=this->goal_point[0]-start_point[0];
            deltay=this->goal_point[1]-start_point[1];
        }
        r=sqrt(deltax*deltax+deltay*deltay);
        if(deltay>0)
            Y[i]=acos(deltax/r);
        else
            Y[i]=-acos(deltax/r);
    }
    return Y;
};
double *APF::compute_attraction(double *start_point,double *att_angle)
{
    double R=(this->goal_point[0]-start_point[0])*(this->goal_point[0]-start_point[0])+(this->goal_point[1]-start_point[1])*(this->goal_point[1]-start_point[1]);
    double r=sqrt(R);
    static double Yatt[2];
    Yatt[0]=this->Attraction_K*r*cos(att_angle[0]);
    Yatt[1]=this->Attraction_K*r*sin(att_angle[0]);
    return Yatt;
}
double *APF::compute_repulsion(double *start_point,double *angle,int n)
{
    double *YY=new double[4];
    double Rat=(start_point[0]-this->goal_point[0])*(start_point[0]-this->goal_point[0])+(start_point[1]-this->goal_point[1])*(start_point[1]-this->goal_point[1]);
    double rat=sqrt(Rat);
    double Rre,rre,Yrer,Yata;
    double *Yrerx=new double[n],*Yrery=new double[n],*Yatax=new double[n],*Yatay=new double[n];
    for(int i=0;i<n;i++)
    {
        Rre=(start_point[0]-this->obstacles[i*2+0])*(start_point[0]-this->obstacles[i*2+0])+(start_point[1]-this->obstacles[i*2+1])*(start_point[1]-this->obstacles[i*2+1]);
        rre=sqrt(Rre);
        if(rre>this->Obstacles_dis)
        {
            Yrerx[i]=0;
            Yrery[i]=0;
            Yatax[i]=0;
            Yatay[i]=0;
        }
        else if(rre>this->Obstacles_dis/2)
        {
            Yrer=this->Repulsion_K*(1/rre-1/this->Obstacles_dis)*(1/Rre)*Rat;//分解的Fre1向量
            Yata=this->Repulsion_K*((1/rre-1/this->Obstacles_dis)*(1/rre-1/this->Obstacles_dis))*rat;//分解的Fre2向量       Yata(i)=0;
            Yrerx[i]=Yrer*cos(angle[i+1]+3.1415 );  //angle_re(i)=Y(i+1)
            Yrery[i]=Yrer*sin(angle[i+1]+3.1415 );
            Yatax[i]=Yata*cos(angle[0]);            //angle_at=Y(1)
            Yatay[i]=Yata*sin(angle[0]);
        }
        else if(rre<this->Obstacles_dis/2)
        {
            Yrer=this->Repulsion_K*(1/rre-1/this->Obstacles_dis)*(1/Rre)*(pow(rat,this->a));//分解的Fre1向量
            Yata=this->a*this->Repulsion_K*((1/rre-1/this->Obstacles_dis)*(1/rre-1/this->Obstacles_dis))*(pow(rat,(1-this->a)))/2;//分解的Fre2向量   Yata(i)=0;
            Yrerx[i]=Yrer*cos(angle[i+1]+3.1415);  //angle_re(i)=Y(i+1)
            Yrery[i]=Yrer*sin(angle[i+1]+3.1415);
            Yatax[i]=Yata*cos(angle[0]);            //angle_at=Y(1)
            Yatay[i]=Yata*sin(angle[0]);
        }
    }
    YY[0]=sum(Yrerx,n);
    YY[1]=sum(Yrery,n);
    YY[2]=sum(Yatax,n);
    YY[3]=sum(Yatay,n);
    return YY;
}
double sum(double *p1,int n)
{
    double sum=0;
    for(int i=0;i<n;i++)
    {
        sum+=p1[i];
    }
    return sum;
}
int main()
{
    cout<<"begin"<<endl;
    APF APF1;
    int n=sizeof(obs)/sizeof(double)/2;
    double Attraction_K=30;
    double Repulsion_K=15;
    double Obstacles_dis=5.5;
    double a=0.5;
    double step=0.2;
    double start_point[2]={0,0},goal_point[2]={10,10};
    double *angle_re,*Yatt,*Y;
    APF1.APF_init(Attraction_K,Repulsion_K,Obstacles_dis,a,step,start_point,goal_point,obs);
    double path[200][2];
    int iterator=200;
    int k1=0;
    double Xj[2]={0,0};
    double Xnext[2]={0,0};
    for(int j=0;j<iterator;j++)
    {
        path[j][0]=Xj[0];
        path[j][1]=Xj[1];
        angle_re=APF1.compute_angle(Xj,n);
        Yatt=APF1.compute_attraction(Xj,angle_re);
        Y=APF1.compute_repulsion(Xj,angle_re,n);
        double Fsumyj=Yatt[1]+Y[1]+Y[3];
        double Fsumxj=Yatt[0]+Y[0]+Y[2];
        double Position_angle=atan(Fsumyj/Fsumxj);
        Xnext[0]=Xj[0]+APF1.step*cos(Position_angle);
        Xnext[1]=Xj[1]+APF1.step*sin(Position_angle);
        Xj[0]=Xnext[0];
        Xj[1]=Xnext[1];
        if(fabs(Xj[0]-APF1.goal_point[0])<0.1&&fabs(Xj[1]-APF1.goal_point[1])<0.1)
        {
            path[j+1][0]=Xj[0];
            path[j+1][1]=Xj[1];
            k1=j;
            cout<<"arrived   "<<k1<<endl;
            break;
        }
      
    }
    //这里可以将打印的path_x和path_y输入到matlab命令行窗口，再输入plot(path_x,path_y,'.r'),利用matlab的plot来绘制路径图；
//     cout<<"path_x=[";
//     for(int j=0;j<k1+2;j++)
//     {
//         cout<<path[j][0]<<" ";
//     }
//     cout<<"];"<<"path_y=[";
//     for(int j=0;j<k1+2;j++)
//     {
//         cout<<path[j][1]<<" ";
//     }
//     cout<<"]";
    Mat img(500,500,CV_8UC3,Scalar(255,255,255));
//     namedWindow("draw points");
    Point p(0, 1000);//初始化点坐标为(0,1000),注意笛卡尔坐标与opencv坐标的区别
    for(int j=0;j<k1+2;j++)
    {
        p.x=int(path[j][0]*50);
        p.y=500-int(path[j][1]*50);//注意笛卡尔坐标与opencv坐标的区别
        circle(img, p, 3,Scalar(255,0,0),-1);//第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了蓝色的实心点
    }
    imshow("draw points",img);
    waitKey(0);
    return 0;
}
