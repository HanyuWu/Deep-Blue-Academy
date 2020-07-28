#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        Eigen::MatrixXd Odom_block(3,9);
        Odom_block << Odom(0), Odom(1), Odom(2), 0, 0, 0, 0, 0, 0
                   , 0, 0, 0, Odom(0), Odom(1), Odom(2), 0, 0, 0
                   , 0, 0, 0, 0, 0, 0, Odom(0), Odom(1), Odom(2);
        Eigen::VectorXd scan_array(3);
        scan_array << scan(0), scan(1), scan(2);
        A.block<3,9>(now_len*3, 0) = Odom_block;
        b.segment(now_len*3,3) = scan_array;
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    Eigen::VectorXd correct_array(9);
    correct_array = (A.transpose()*A).inverse()*A.transpose()*b;
    correct_matrix << correct_array(0), correct_array(1), correct_array(2)
                   , correct_array(3), correct_array(4), correct_array(5)
                   , correct_array(6), correct_array(7), correct_array(8);
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
