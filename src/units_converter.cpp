#include <iostream>
#include <vector>

using namespace std;

constexpr double radTodeg(double rad)
{
    return (rad/3.14159)*180;
}

void prettyPrintTransformation(vector<double>& transformation)
{
    for(int i=0;i<3;i++)
        transformation[i]=(int(transformation[i]*100000.0))/100.0;
    for(int i=3;i<6;i++)
        transformation[i]=int(radTodeg(transformation[i])*100)/100.0;
    for(auto x:transformation)
        cout<<x<<" ";
    cout<<endl;
}

int main()
{
    vector<double> transformation_initial={0.0165, 0.0972, 0.0308, 3.1301, -0.0059, 1.6067};
    vector<double> transformation_trans={0.0165205, 0.0964558, 0.0307458, 3.12552, -0.00621489, 1.59359};
    vector<double> transformation_rot={0.0234896, 0.0951905, 0.0201839, 3.12524, 9.49362e-05, 1.59457};
    vector<double> transformation_trans_rot={0.0286139, 0.0898499, 0.0201967, 3.12109, 0.00633344, 1.59424};
    vector<double> transformation_good={0.0295665, 0.0911525, 0.0202411, 3.12168, 0.0060725, 1.59351};

    prettyPrintTransformation(transformation_initial);
    prettyPrintTransformation(transformation_trans);
    prettyPrintTransformation(transformation_rot);
    prettyPrintTransformation(transformation_trans_rot);
    prettyPrintTransformation(transformation_good);
    return 0;
}

