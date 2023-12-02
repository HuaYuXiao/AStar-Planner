//
// Created by 华羽霄 on 2023/11/24.
//

#include"ArtificialPotential.h"

#if 0
void test_for_RoadF()//道路斥力测试
{
	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);
	setfillcolor(BLACK);
	setorigin(400, 100);

	double e = 2.71828;
	double Eta = 0.02;
	double v = 10.0;
	double x = 0.0;
	double Fedge = 0.0;

	while (-300.0 < x && x < 300.0)
	{
		if (-200.0 <= x && x <= 200.0)
		{
			Fedge = Eta * pow(x, 2) / 3;
		}
		else if (-300.0 < x && x < 300.0)
		{

			Fedge = Eta * v * pow(e, abs(x));
		}
		solidcircle(x, Fedge, 3);
		solidcircle(-x, Fedge, 3);
		cout << "(" << x << ", " << Fedge << ")" << endl;
		x += 0.5;
	}
}

void testVector()//向量计算与绘制测试
{
	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);
	setfillcolor(BLACK);

	setorigin(0, Swidth / 2);
	Point p0(0, 0);
	Point p1(200, 300);
	Point p2(400, -100);
	p0.drawoint();
	p1.drawoint();
	p2.drawoint();

	Point pt(p2.x + p1.x - p0.x, p2.y + p1.y - p0.y);
	pt.drawoint();

	Point p3(750, -150);
	p3.drawoint();
	Point pt1(p3.x + p2.x - pt.x, p3.y + p2.y - pt.y);
	pt1.drawoint();

	system("pause");
	cleardevice();

	setorigin(Swidth / 2, Sheight);
	Point p4(0.0, 0.0);
	Point p5(200.5, -300.5);
	Point p6(-300.5, -400.5);

	newVector v1(p4, p5);
	newVector v2(p4, p6);
	newVector vt = v1.addVector(v1, v2);
}
#endif

void Point::drawoint(COLORREF color)//画点
{
	setfillcolor(color);
	solidcircle(x, y, r);
}

newVector newVector::addVector(newVector v1, newVector v2)//向量求和
{
	Point pEndtmp(v2.pEnd.x + v1.pEnd.x - v1.pBegin.x, v2.pEnd.y + v1.pEnd.y - v1.pBegin.y);
	newVector vt(v1.pBegin, pEndtmp);
	return vt;
}

void newVector::drawVector()//绘制向量
{
	pBegin.drawoint(BLACK);
	pEnd.drawoint(BLACK);
	setlinestyle(PS_SOLID, 3);
	setlinecolor(LIGHTRED);
	line(pBegin.x, pBegin.y, pEnd.x, pEnd.y);
}

void newVector::printVector(string name)//打印向量
{
	cout << name
		<< ": pBegin: (" << pBegin.x << ", " << pBegin.y << ")"
		<< ", pEnd: (" << pEnd.x << ", " << pEnd.y << ")"
		<< ", length: " << length << endl;
}

void Obs::drawObs()//绘制障碍物
{
	setfillcolor(RED);
	fillcircle(po.x, po.y, r);
}

void Car::drawCar(Point p)//绘制车辆
{
	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);

	double leftPos = p.x - width / 2;
	double rightPos = p.x + width / 2;
	double topPos = p.y - length / 2;
	double bottomPos = p.y + length / 2;

	rectangle(leftPos, topPos, rightPos, bottomPos);
}

Road::Road()
{
	car0 = new Car(Point(0.0, -80.0));//定义车辆
	pTarget = new Point(-20.0, -1100.0);//定义终点
	cout << "pTarget: (" << pTarget->x << ", " << pTarget->y << ")" << endl;

	if (bObs)
	{
		obTotal.push_back(Obs(Point(-100.0, -250.0)));
		obTotal.push_back(Obs(Point(0.0, -460.0)));
		obTotal.push_back(Obs(Point(100.0, -660.0)));
		obTotal.push_back(Obs(Point(-60.0, -830.0)));
		obTotal.push_back(Obs(Point(50.0, -1000.0)));
	}

	showRoad(car0->pc);//绘制道路
	pTarget->drawoint(GREEN);//绘制终点
	system("pause");
}

Road::~Road()
{
	if (car0 != nullptr)
	{
		delete car0;
		car0 = nullptr;
	}

	if (pTarget != nullptr)
	{
		delete pTarget;
		pTarget = nullptr;
	}
}

void Road::showRoad(Point p)//绘制道路
{
	setlinecolor(BLACK);
	setorigin(Swidth / 2, Sheight);

	//绘制道路中心
	setlinestyle(PS_DASH, 5);
	line(0, 0, 0, -Sheight);

	//绘制左右边界
	setlinestyle(PS_SOLID, 5);
	line(-RWidth, 0, -RWidth, -Sheight);
	line(RWidth, 0, RWidth, -Sheight);

	for (auto it = obTotal.begin(); it != obTotal.end(); it++)
	{
		it->drawObs();//绘制障碍物
	}

	car0->drawCar(p);//绘制车辆
}

double ArtificialPotential::disCal(Point p1, Point p2)//两点距离计算
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

newVector ArtificialPotential::Fatt()//计算引力
{
	Rho_att = disCal(*road.pTarget, road.car0->pc);//与目标点距离
	double F_att = eta * Rho_att;//引力大小
	Point pFatt(F_att / Rho_att * (road.pTarget->x - road.car0->pc.x) + road.car0->pc.x, F_att / Rho_att * (road.pTarget->y - road.car0->pc.y) + road.car0->pc.y);
	newVector vFatt(road.car0->pc, pFatt);

	vFatt.printVector("vFatt");
	//vFatt.drawVector();

	return vFatt;
}

newVector ArtificialPotential::Fobs()//计算斥力
{
	if (road.obTotal.empty())//没有障碍物
	{
		return newVector (road.car0->pc, road.car0->pc);
	}

	for (auto it = road.obTotal.begin(); it != road.obTotal.end(); it++)//分别计算每个障碍物产生的合斥力
	{
		double Rho_obs = disCal(it->po, road.car0->pc);//与这个障碍物中心点的距离
		double Freo1 = 0.0;
		double Freo2 = 0.0;//这个障碍物产生的两个引力的大小

		if (Rho_obs >= Rho0)
		{
			Freo1 = 0.0;
			Freo2 = 0.0;
		}
		else
		{
			Freo1 = k * (1 / Rho_obs - 1 / Rho0) * pow(Rho_att, n) / pow(Rho_obs, 2);
			Freo2 = n * k / 2 * pow(1 / Rho_obs - 1 / Rho0, 2) * pow(Rho_att, n - 1);
		}

		Point pFreo1((road.car0->pc.x - it->po.x) / Rho_obs * (Freo1 + Rho_obs) + it->po.x, (road.car0->pc.y - it->po.y) / Rho_obs * (Freo1 + Rho_obs) + it->po.y);//由这个障碍物指向车辆
		Point pFreo2(Freo2 / Rho_att * (road.pTarget->x - road.car0->pc.x) + road.car0->pc.x, Freo2 / Rho_att * (road.pTarget->y - road.car0->pc.y) + road.car0->pc.y);//由车辆指向目标点
		newVector vFreo1(road.car0->pc, pFreo1);
		newVector vFreo2(road.car0->pc, pFreo2);
		newVector vFobs = vFreo1.addVector(vFreo1, vFreo2);//计算这个障碍物的合斥力
		vFreo.push_back(vFobs);//存入容器

		vFreo1.printVector("vFreo1");
		vFreo2.printVector("vFreo2");
		vFobs.printVector("vFobs");
		//vFreo1.drawVector();
		//vFreo2.drawVector();
		//vFobs.drawVector();
	}

	//计算所有障碍物产生的合斥力
	newVector vFobsTotal = vFreo[0];
	for (int i = 1; i < vFreo.size(); i++)
	{
		vFobsTotal = vFobsTotal.addVector(vFobsTotal, vFreo[i]);
	}
	vFobsTotal.printVector("vFobsTotal");
	//vFobsTotal.drawVector();
	return vFobsTotal;
}

newVector ArtificialPotential::Fedge()//计算道路边界斥力
{
	if (!bEdgeF)//不考虑道路边界斥力
	{
		return newVector(road.car0->pc, road.car0->pc);
	}

	double Fedge = 0.0;
	Point pFedge(0.0, 0.0);
	if (-(road.RWidth - road.car0->width / 2) <= road.car0->pc.x && road.car0->pc.x < 0.0)
	{
		Fedge = Eta * pow(road.car0->pc.x, 2) / 3;
		pFedge = Point(road.car0->pc.x + Fedge, road.car0->pc.y);
	}
	else if (road.car0->pc.x >= 0.0 && road.car0->pc.x <= (road.RWidth - road.car0->width / 2))
	{
		Fedge = Eta * pow(road.car0->pc.x, 2) / 3;
		pFedge = Point(road.car0->pc.x - Fedge, road.car0->pc.y);
	}
	else if (-road.RWidth < road.car0->pc.x && road.car0->pc.x < -(road.RWidth - road.car0->width / 2))
	{
		Fedge = Eta * v0 * pow(e, -road.car0->pc.x);
		pFedge = Point(road.car0->pc.x + Fedge, road.car0->pc.y);
	}
	else if ((road.RWidth - road.car0->width / 2) < road.car0->pc.x && road.car0->pc.x < road.RWidth)
	{
		Fedge = Eta * v0 * pow(e, road.car0->pc.x);
		pFedge = Point(road.car0->pc.x - Fedge, road.car0->pc.y);
	}

	newVector vFedge(road.car0->pc, pFedge);
	vFedge.printVector("vFedge");
	//vFedge.drawVector();

	return vFedge;
}

void ArtificialPotential::process()//整个过程
{
	while (true)
	{
		newVector vFatt = Fatt();//引力
		newVector vFobsTotal = Fobs();//总障碍物斥力
		newVector vFedge = Fedge();//道路边界斥力
		newVector vFtotal = vFatt.addVector(vFatt, vFobsTotal);//引力与斥力的合力
		vFtotal = vFtotal.addVector(vFtotal, vFedge);//再与道路斥力计算合力
		vFtotal.printVector("vFtotal");
		//vFtotal.drawVector();

		double dis = v0 * delta_t + vFtotal.length * pow(delta_t, 2) / 2;//每个时间间隔内走过的距离
		v0 += vFtotal.length * delta_t;//速度
		road.car0->pc.x = dis / vFtotal.length * (vFtotal.pEnd.x - vFtotal.pBegin.x) + vFtotal.pBegin.x;
		road.car0->pc.y = dis / vFtotal.length * (vFtotal.pEnd.y - vFtotal.pBegin.y) + vFtotal.pBegin.y;//更新车辆坐标位置
		road.car0->pc.drawoint(BLUE);

		cout << "CarLoc: (" << road.car0->pc.x << ", " << road.car0->pc.y << "), dis to goal: " << Rho_att << ", dis: " << dis << ", v: " << v0 << endl << endl;
		//road.car0->drawCar(road.car0->pc);
		vFreo.clear();

		if (Rho_att <= road.car0->length / 2)//已到达目标点
		{
			break;
		}

		//delay(100);
		//system("pause");
	}
}

void ArtificialPotential::delay(int time) //延时函数，单位ms
{
	clock_t  now = clock();
	while (clock() - now < time)
	{

	}
}


int main()
{
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);
	setbkcolor(WHITE);
	cleardevice();

	ArtificialPotential ap;
	ap.process();


	system("pause");
	closegraph();
	return 0;
}
