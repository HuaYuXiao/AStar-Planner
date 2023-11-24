//
// Created by 华羽霄 on 2023/11/24.
//

#pragma once
#include<iostream>
#include<easyX.h>
#include<cmath>
#include<ctime>
#include<vector>
#include<string>
using namespace std;

constexpr auto Swidth = 800;
constexpr auto Sheight = 1200;
constexpr auto e = 2.71828;

class Point//点
{
public:
	Point(int _x, int _y) :x(_x), y(_y)//初始化列表
	{

	}

public:
	void drawoint(COLORREF color);//画点

	double x;
	double y;
	int r = 5;//画图半径
};

class newVector//向量
{
public:
	newVector(Point p1, Point p2) :pBegin(p1), pEnd(p2)//, length(len)//初始化列表
	{
		length = sqrt(pow((pEnd.x - pBegin.x), 2) + pow(pEnd.y - pBegin.y, 2));//计算长度
		//drawVector();
	}

	newVector addVector(newVector v1, newVector v2);//向量求和
	void drawVector();//绘制向量
	void printVector(string name);//打印向量

public:
	Point pBegin;
	Point pEnd;
	double length;
};

class Obs//障碍物
{
public:
	Obs(Point p) :po(p)//初始化列表
	{
		cout << "ObsLoc: (" << po.x << ", " << po.y << ")" << endl;
	}

	void drawObs();//绘制障碍物

public:
	Point po;
	double r = 20.0;//半径
};

class Car//车辆
{
public:
	Car(Point p) :pc(p)
	{
		cout << "CarLoc: (" << pc.x << ", " << pc.y << ")" << endl;
	}

	void drawCar(Point p);//绘制车辆

public:
	double length = 100.0;//长度
	double width = 50.0;//宽度
	Point pc;//中心点
};

class Road//道路
{
public:
	Road();
	~Road();
	void showRoad(Point p);//绘制道路

public:
	double RWidth = 200.0;
	vector<Obs> obTotal;//所有障碍物
	Car *car0;//车辆
	Point *pTarget;//终点
	bool bObs = true;//障碍物开关
};


class ArtificialPotential//人工势场法
{
public:
	double disCal(Point p1, Point p2);//两点距离计算
	newVector Fatt();//计算引力
	newVector Fobs();//计算障碍物斥力
	newVector Fedge();//计算道路边界斥力
	void process();//整个过程
	void delay(int time); //延时函数，单位ms

public:
	Road road;
	double delta_t = 0.02;//时间步长
	double Rho_att;//与目标点的距离
	double eta = 0.3;//引力增益系数//已考虑车辆质量，算出的力直接当作加速度用
	double k = 0.3;//障碍物斥力增益系数//已考虑车辆质量，算出的力直接当作加速度用
	int n = 3;
	double Rho0 = 500.0;//斥力起作用的范围
	vector<newVector> vFreo;//存储每个障碍物产生的合斥力
	double v0 = 0.0;//起始速度为0
	double Eta = 0.02;//道路边界斥力增益系数
	bool bEdgeF = true;//道路边界斥力开关
};
