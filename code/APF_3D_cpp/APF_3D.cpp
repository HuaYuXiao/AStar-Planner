#include <iostream>
#include <cmath>


using namespace std;


double obstacles[21] = {1, 1.2, 3,
                  2.5, 4, 4.5,
                  3, 6, 6,
                  2, 5.5, 5.5,
                  8, 8.5, 1,
                  1, 5, 6,
                  2, 7, 8};

double sum(double *p1, int n) {
    double sum = 0;
    for (int i = 0; i < n; i++) {
        sum += p1[i];
    }
    return sum;
}


double RSS(double x, double y, double z){
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
double RSS(double *a, double *b, int i=0) {
    return RSS(a[0] - b[i*3+0], a[1] - b[i*3+1], a[2] - b[i*3+2]);
}


class APF {
public:
    double Attraction_K;
    double Repulsion_K;
    double Obstacles_dis;
    double a;
    double step;
    double *startPoint;
    double *goalPoint;
    double *obstacles;
    int n;

    void __init__(double Attraction_K, double Repulsion_K, double Obstacles_dis, double a, double step,
                  double *startPoint, double *goalPoint, double *obstacles) {
        this->Attraction_K = Attraction_K;
        this->Repulsion_K = Repulsion_K;
        this->Obstacles_dis = Obstacles_dis;
        this->a = a;
        this->step = step;
        this->startPoint = startPoint;
        this->goalPoint = goalPoint;
        this->obstacles = obstacles;
        this->n = sizeof(obstacles) / sizeof(double) / 3;
    }

    double *computeAngle(double *startPoint);

    double *compute_attraction(double *startPoint, double *attAngle);

    double *compute_repulsion(double *startPoint, double *angle);
};


double *APF::computeAngle(double *startPoint) {
    double *Y = new double[this->n + 1];
    double deltaX, deltaY, deltaZ;
    for (int i = 0; i < this->n + 1; i++) {
        double r;
        if (i != 0) {
            deltaX = this->obstacles[(i - 1) * 3 + 0] - startPoint[0];
            deltaY = this->obstacles[(i - 1) * 3 + 1] - startPoint[1];
            deltaZ = this->obstacles[(i - 1) * 3 + 2] - startPoint[2];
            r = RSS(startPoint, this->obstacles, i - 1);
        } else {
            deltaX = this->goalPoint[0] - startPoint[0];
            deltaY = this->goalPoint[1] - startPoint[1];
            deltaZ = this->goalPoint[2] - startPoint[2];
            r = RSS(startPoint, this->obstacles);
        }
        if (deltaY > 0)
            Y[i] = acos(deltaX / r);
        else
            Y[i] = -acos(deltaX / r);
    }
    return Y;
}


double *APF::compute_attraction(double *startPoint, double *attAngle) {
    double r = RSS(this->goalPoint, startPoint);
    static double Yatt[3];
    Yatt[0] = this->Attraction_K * r * cos(attAngle[0]);
    Yatt[1] = this->Attraction_K * r * sin(attAngle[0]);
    Yatt[2] = this->Attraction_K * r * sin(attAngle[0]);
    return Yatt;
}


double *APF::compute_repulsion(double *startPoint, double *angle) {
    double *YY = new double[6];
    double rat = RSS(startPoint, this->goalPoint);
    double Yrer, Yata;
    double *Yrerx = new double[this->n],
           *Yrery = new double[this->n],
           *Yatax = new double[this->n],
           *Yatay = new double[this->n],
           *Yataz = new double[this->n];
    for (int i = 0; i < this->n; i++) {
        double rre = RSS(startPoint, this->obstacles, i);
        if (rre > this->Obstacles_dis) {
            Yrerx[i] = 0;
            Yrery[i] = 0;
            Yatax[i] = 0;
            Yatay[i] = 0;
            Yataz[i] = 0;  // Add a third component for the 3D case
        } else if (rre > this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / pow(rre, 2)) * pow(rat, 2);
            Yata = this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) * rat;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
            Yataz[i] = 0;
        } else if (rre < this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / pow(rre, 2)) * (pow(rat, this->a));
            Yata =
                this->a * this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) *
                (pow(rat, (1 - this->a))) / 2;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
            Yataz[i] = 0;
        }
    }
    YY[0] = sum(Yrerx, this->n);
    YY[1] = sum(Yrery, this->n);
    YY[2] = sum(Yatax, this->n);
    YY[3] = sum(Yatay, this->n);
    YY[4] = sum(Yataz, this->n);
    YY[5] = 0;
    return YY;
}


int main() {
    APF myAPF{};

    double Attraction_K = 30;
    double Repulsion_K = 15;
    double Obstacles_dis = 5.5;
    double a = 0.5;
    double step = 0.2;
    double startPoint[3] = {0, 0, 0};
    double goalPoint[3] = {10, 10, 10};
    double *angle_re, *Yatt, *Y;

    myAPF.__init__(Attraction_K, Repulsion_K, Obstacles_dis, a, step, startPoint, goalPoint, obstacles);
    double path[200][3];
    int iterator = 200;

    double Xj[3] = {0, 0, 0};
    double Xnext[3] = {0, 0, 0};

    for (int j = 0; j < iterator; j++) {
        path[j][0] = Xj[0];
        path[j][1] = Xj[1];
        path[j][2] = Xj[2];
        angle_re = myAPF.computeAngle(Xj);
        Yatt = myAPF.compute_attraction(Xj, angle_re);
        Y = myAPF.compute_repulsion(Xj, angle_re);
        double Fsumyj = Yatt[1] + Y[1] + Y[3];
        double Fsumxj = Yatt[0] + Y[0] + Y[2];
        double Fsumzj = Yatt[2] + Y[4] + Y[5];
        double PositionAngle = atan2(Fsumyj, Fsumxj);
        Xnext[0] = Xj[0] + myAPF.step * cos(PositionAngle);
        Xnext[1] = Xj[1] + myAPF.step * sin(PositionAngle);
        Xnext[2] = Xj[2] + myAPF.step * sin(PositionAngle);
        Xj[0] = Xnext[0];
        Xj[1] = Xnext[1];
        Xj[2] = Xnext[2];

        if (fabs(Xj[0] - myAPF.goalPoint[0]) < 0.1 &&
            fabs(Xj[1] - myAPF.goalPoint[1]) < 0.1 &&
            fabs(Xj[2] - myAPF.goalPoint[2]) < 0.1) {
            path[j + 1][0] = Xj[0];
            path[j + 1][1] = Xj[1];
            path[j + 1][2] = Xj[2];
            break;
        }
    }
    return 0;
}
