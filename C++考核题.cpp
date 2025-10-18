#include<iostream>
#include<windows.h>
#include<cmath>
#include<stdio.h>
#include<string>
using namespace std;

//定义rect结构体
struct Rect {
    int id;//数字id
    int color;//颜色
    int l_point, r_point;//横纵坐标分开列
    int width;//宽
    int height;//高
};

class Armor {

public:

    void Central_Point(int long1, int kuan, int h1, int gao) {             //计算中心坐标
        int central_l = (long1 + long1 + kuan) / 2;
        int central_r = (h1 + h1 + gao) / 2;
        cout << "(" << central_l << "," << central_r << ")";
    }

    void Diagonal(int kuan2, int gao2) {                                //计算对角线长度
        float duijiao = sqrt(kuan2 * kuan2 + gao2 * gao2);                  //对角线长度公式
        printf("%.2f\n", duijiao);   //printf格式化输出使对角线长度保留两位小数
    }

    void Armor_Point(int lp, int rp, int kuan3, int gao3) {                 //输出4点坐标（左上开始顺时针)
        cout << "(" << lp << "," << rp << ")";
        cout << "(" << lp + kuan3 << "," << rp << ")";
        cout << "(" << lp + kuan3 << "," << rp + gao3 << ")";
        cout << "(" << lp << "," << rp + gao3 << ")";
    }

    void Armor_Colour(int color) {                                      //输出颜色
        const string lan = "颜色：蓝";
        const string hong = "颜色：红";

        if (color == 0)
            cout << lan;
        else
            cout << hong;
    }
};

int main() {
    SetConsoleOutputCP(CP_UTF8);                            //用utf编码防止中文乱码(但没有用）
    Rect Rect1;

    cin >> Rect1.id >> Rect1.color;
    cin >> Rect1.l_point >> Rect1.r_point;
    cin >> Rect1.width >> Rect1.height;

    Armor Armor1;

    cout << "ID:" << Rect1.id;
    Armor1.Armor_Colour(Rect1.color);
    cout << endl;
    Armor1.Central_Point(Rect1.l_point, Rect1.width, Rect1.r_point, Rect1.height);
    Armor1.Diagonal(Rect1.width, Rect1.height);
    Armor1.Armor_Point(Rect1.l_point, Rect1.r_point, Rect1.width, Rect1.height);
    return 0;
}