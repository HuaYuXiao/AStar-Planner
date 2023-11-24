#include <iostream>
#include <cmath>


using namespace std;


double obs[21] = {1, 1.2, 3,
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


class APF {
public:
    double Attraction_K;
    double Repulsion_K;
    double Obstacles_dis;
    double a;
    double step;
    double *start_point;
    double *goal_point;
    double *obstacles;

    void __init__(double Attraction_K, double Repulsion_K, double Obstacles_dis, double a, double step,
                  double *start_point, double *goal_point, double *obstacles) {
        this->Attraction_K = Attraction_K;
        this->Repulsion_K = Repulsion_K;
        this->Obstacles_dis = Obstacles_dis;
        this->a = a;
        this->step = step;
        this->start_point = start_point;
        this->goal_point = goal_point;
        this->obstacles = obstacles;
    }

    double *compute_angle(double *start_point, int n);

    double *compute_attraction(double *start_point, double *att_angle);

    double *compute_repulsion(double *start_point, double *angle, int n);
};

double *APF::compute_angle(double *start_point, int n) {
    double *Y = new double[n + 1];
    double deltax, deltay, deltaz, r;
    for (int i = 0; i < n + 1; i++) {
        if (i != 0) {
            deltax = this->obstacles[(i - 1) * 3] - start_point[0];
            deltay = this->obstacles[(i - 1) * 3 + 1] - start_point[1];
            deltaz = this->obstacles[(i - 1) * 3 + 2] - start_point[2];
        } else {
            deltax = this->goal_point[0] - start_point[0];
            deltay = this->goal_point[1] - start_point[1];
            deltaz = this->goal_point[2] - start_point[2];
        }
        r = sqrt(deltax * deltax + deltay * deltay + deltaz * deltaz);
        if (deltay > 0)
            Y[i] = acos(deltax / r);
        else
            Y[i] = -acos(deltax / r);
    }
    return Y;
}


double *APF::compute_attraction(double *start_point, double *att_angle) {
    double R = (this->goal_point[0] - start_point[0]) * (this->goal_point[0] - start_point[0]) +
               (this->goal_point[1] - start_point[1]) * (this->goal_point[1] - start_point[1]) +
               (this->goal_point[2] - start_point[2]) * (this->goal_point[2] - start_point[2]);
    double r = sqrt(R);
    static double Yatt[3];
    Yatt[0] = this->Attraction_K * r * cos(att_angle[0]);
    Yatt[1] = this->Attraction_K * r * sin(att_angle[0]);
    Yatt[2] = this->Attraction_K * r * sin(att_angle[0]);  // Add a third component for the 3D case
    return Yatt;
}


double *APF::compute_repulsion(double *start_point, double *angle, int n) {
    double *YY = new double[6];  // Expand to six components for the 3D case
    double Rat = (start_point[0] - this->goal_point[0]) * (start_point[0] - this->goal_point[0]) +
                  (start_point[1] - this->goal_point[1]) * (start_point[1] - this->goal_point[1]) +
                  (start_point[2] - this->goal_point[2]) * (start_point[2] - this->goal_point[2]);
    double rat = sqrt(Rat);
    double Rre, rre, Yrer, Yata;
    double *Yrerx = new double[n], *Yrery = new double[n], *Yatax = new double[n], *Yatay = new double[n],
           *Yataz = new double[n];  // Add a third component for the 3D case
    for (int i = 0; i < n; i++) {
        Rre = (start_point[0] - this->obstacles[i * 3 + 0]) * (start_point[0] - this->obstacles[i * 3 + 0]) +
              (start_point[1] - this->obstacles[i * 3 + 1]) * (start_point[1] - this->obstacles[i * 3 + 1]) +
              (start_point[2] - this->obstacles[i * 3 + 2]) * (start_point[2] - this->obstacles[i * 3 + 2]);
        rre = sqrt(Rre);
        if (rre > this->Obstacles_dis) {
            Yrerx[i] = 0;
            Yrery[i] = 0;
            Yatax[i] = 0;
            Yatay[i] = 0;
            Yataz[i] = 0;  // Add a third component for the 3D case
        } else if (rre > this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / Rre) * Rat;
            Yata = this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) * rat;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
            Yataz[i] = 0;  // Add a third component for the 3D case
        } else if (rre < this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / Rre) * (pow(rat, this->a));
            Yata =
                this->a * this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) *
                (pow(rat, (1 - this->a))) / 2;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
            Yataz[i] = 0;  // Add a third component for the 3D case
        }
    }
    YY[0] = sum(Yrerx, n);
    YY[1] = sum(Yrery, n);
    YY[2] = sum(Yatax, n);
    YY[3] = sum(Yatay, n);
    YY[4] = sum(Yataz, n);  // Add a fifth component for the 3D case
    YY[5] = 0;              // Set the sixth component to 0 for the 3D case
    return YY;
}


int main() {
    APF myAPF{};
    int n = sizeof(obs) / sizeof(double) / 3;
    double Attraction_K = 30;
    double Repulsion_K = 15;
    double Obstacles_dis = 5.5;
    double a = 0.5;
    double step = 0.2;
    double start_point[3] = {0, 0, 0};
    double goal_point[3] = {10, 10, 10};
    double *angle_re, *Yatt, *Y;

    myAPF.__init__(Attraction_K, Repulsion_K, Obstacles_dis, a, step, start_point, goal_point, obs);
    double path[200][3];
    int iterator = 200;

    double Xj[3] = {0, 0, 0};
    double Xnext[3] = {0, 0, 0};

    for (int j = 0; j < iterator; j++) {
        path[j][0] = Xj[0];
        path[j][1] = Xj[1];
        path[j][2] = Xj[2];
        angle_re = myAPF.compute_angle(Xj, n);
        Yatt = myAPF.compute_attraction(Xj, angle_re);
        Y = myAPF.compute_repulsion(Xj, angle_re, n);
        double Fsumyj = Yatt[1] + Y[1] + Y[3];
        double Fsumxj = Yatt[0] + Y[0] + Y[2];
        double Fsumzj = Yatt[2] + Y[4] + Y[5];
        double Position_angle = atan2(Fsumyj, Fsumxj);
        Xnext[0] = Xj[0] + myAPF.step * cos(Position_angle);
        Xnext[1] = Xj[1] + myAPF.step * sin(Position_angle);
        Xnext[2] = Xj[2] + myAPF.step * sin(Position_angle);
        Xj[0] = Xnext[0];
        Xj[1] = Xnext[1];
        Xj[2] = Xnext[2];

        if (fabs(Xj[0] - myAPF.goal_point[0]) < 0.1 && fabs(Xj[1] - myAPF.goal_point[1]) < 0.1 && fabs(Xj[2] - myAPF.goal_point[2]) < 0.1) {
            path[j + 1][0] = Xj[0];
            path[j + 1][1] = Xj[1];
            path[j + 1][2] = Xj[2];

            break;
        }
    }

    return 0;
}
