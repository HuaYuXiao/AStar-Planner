#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

double obs[14] = {1, 1.2, 3, 2.5, 4, 4.5, 3, 6, 6, 2, 5.5, 5.5, 8, 8.5};

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

    void APF_init(double Attraction_K, double Repulsion_K, double Obstacles_dis, double a, double step,
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
    double deltax, deltay, r;
    for (int i = 0; i < n + 1; i++) {
        if (i != 0) {
            deltax = this->obstacles[(i - 1) * 2] - start_point[0];
            deltay = this->obstacles[(i - 1) * 2 + 1] - start_point[1];
        } else {
            deltax = this->goal_point[0] - start_point[0];
            deltay = this->goal_point[1] - start_point[1];
        }
        r = sqrt(deltax * deltax + deltay * deltay);
        if (deltay > 0)
            Y[i] = acos(deltax / r);
        else
            Y[i] = -acos(deltax / r);
    }
    return Y;
}

double *APF::compute_attraction(double *start_point, double *att_angle) {
    double R = (this->goal_point[0] - start_point[0]) * (this->goal_point[0] - start_point[0]) +
               (this->goal_point[1] - start_point[1]) * (this->goal_point[1] - start_point[1]);
    double r = sqrt(R);
    static double Yatt[2];
    Yatt[0] = this->Attraction_K * r * cos(att_angle[0]);
    Yatt[1] = this->Attraction_K * r * sin(att_angle[0]);
    return Yatt;
}

double *APF::compute_repulsion(double *start_point, double *angle, int n) {
    double *YY = new double[4];
    double Rat = (start_point[0] - this->goal_point[0]) * (start_point[0] - this->goal_point[0]) +
                  (start_point[1] - this->goal_point[1]) * (start_point[1] - this->goal_point[1]);
    double rat = sqrt(Rat);
    double Rre, rre, Yrer, Yata;
    double *Yrerx = new double[n], *Yrery = new double[n], *Yatax = new double[n], *Yatay = new double[n];
    for (int i = 0; i < n; i++) {
        Rre = (start_point[0] - this->obstacles[i * 2 + 0]) * (start_point[0] - this->obstacles[i * 2 + 0]) +
              (start_point[1] - this->obstacles[i * 2 + 1]) * (start_point[1] - this->obstacles[i * 2 + 1]);
        rre = sqrt(Rre);
        if (rre > this->Obstacles_dis) {
            Yrerx[i] = 0;
            Yrery[i] = 0;
            Yatax[i] = 0;
            Yatay[i] = 0;
        } else if (rre > this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / Rre) * Rat;
            Yata = this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) * rat;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
        } else if (rre < this->Obstacles_dis / 2) {
            Yrer = this->Repulsion_K * (1 / rre - 1 / this->Obstacles_dis) * (1 / Rre) * (pow(rat, this->a));
            Yata = this->a * this->Repulsion_K * ((1 / rre - 1 / this->Obstacles_dis) * (1 / rre - 1 / this->Obstacles_dis)) *
                   (pow(rat, (1 - this->a))) / 2;
            Yrerx[i] = Yrer * cos(angle[i + 1] + 3.1415);
            Yrery[i] = Yrer * sin(angle[i + 1] + 3.1415);
            Yatax[i] = Yata * cos(angle[0]);
            Yatay[i] = Yata * sin(angle[0]);
        }
    }
    YY[0] = sum(Yrerx, n);
    YY[1] = sum(Yrery, n);
    YY[2] = sum(Yatax, n);
    YY[3] = sum(Yatay, n);
    return YY;
}

int main() {
    APF APF1;
    int n = sizeof(obs) / sizeof(double) / 2;
    double Attraction_K = 30;
    double Repulsion_K = 15;
    double Obstacles_dis = 5.5;
    double a = 0.5;
    double step = 0.2;
    double start_point[2] = {0, 0}, goal_point[2] = {10, 10};
    double *angle_re, *Yatt, *Y;

    APF1.APF_init(Attraction_K, Repulsion_K, Obstacles_dis, a, step, start_point, goal_point, obs);
    double path[200][2];
    int iterator = 200;
    int k1 = 0;
    double Xj[2] = {0, 0};
    double Xnext[2] = {0, 0};

    Mat img(500, 500, CV_8UC3, Scalar(255, 255, 255));
    Point p(0, 1000);

    // 绘制障碍物
    for (int i = 0; i < n; i++) {
        p.x = int(obs[i * 2] * 50);
        p.y = 500 - int(obs[i * 2 + 1] * 50);
        circle(img, p, 5, Scalar(0, 0, 255), -1);
    }

    // 模拟
    for (int j = 0; j < iterator; j++) {
        path[j][0] = Xj[0];
        path[j][1] = Xj[1];
        angle_re = APF1.compute_angle(Xj, n);
        Yatt = APF1.compute_attraction(Xj, angle_re);
        Y = APF1.compute_repulsion(Xj, angle_re, n);
        double Fsumyj = Yatt[1] + Y[1] + Y[3];
        double Fsumxj = Yatt[0] + Y[0] + Y[2];
        double Position_angle = atan(Fsumyj / Fsumxj);
        Xnext[0] = Xj[0] + APF1.step * cos(Position_angle);
        Xnext[1] = Xj[1] + APF1.step * sin(Position_angle);
        Xj[0] = Xnext[0];
        Xj[1] = Xnext[1];

        // 是否到达目标点
        if (fabs(Xj[0] - APF1.goal_point[0]) < 0.1 && fabs(Xj[1] - APF1.goal_point[1]) < 0.1) {
            path[j + 1][0] = Xj[0];
            path[j + 1][1] = Xj[1];
            k1 = j;
            break;
        }
    }

    // 绘制路径
    for (int j = 0; j < k1 + 2; j++) {
        p.x = int(path[j][0] * 50);
        p.y = 500 - int(path[j][1] * 50);
        circle(img, p, 3, Scalar(255, 0, 0), -1);
    }

    imshow("绘制路径和障碍物", img);
    waitKey(0);
    return 0;
}
