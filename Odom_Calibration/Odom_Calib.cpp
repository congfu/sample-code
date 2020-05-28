#include "../include/calib_odom/Odom_Calib.hpp"

// set data size
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}

// use odom and laser scan data to construct overdetermined equation
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        // construct overdetermined equation
        A(now_len*3, 0) = Odom(0);
        A(now_len*3, 1) = Odom(1);
        A(now_len*3, 2) = Odom(2);
        A(now_len*3+1, 3) = Odom(0);
        A(now_len*3+1, 4) = Odom(1);
        A(now_len*3+1, 5) = Odom(2);
        A(now_len*3+2, 6) = Odom(0);
        A(now_len*3+2, 7) = Odom(1);
        A(now_len*3+2, 8) = Odom(2);

        b(now_len*3) = scan(0);
        b(now_len*3+1) = scan(1);
        b(now_len*3+2) = scan(2);

        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

// solve least square to get calibration matrix
Eigen::Matrix3d OdomCalib::Solve()
{
    std::cout << "solve!!!!!!!!!!!" << std::endl;
    Eigen::Matrix3d correct_matrix;

    // solve least square
    Eigen::Matrix<double, 9, 1> X;
    X = A.colPivHouseholderQr().solve(b);
    correct_matrix << X(0), X(1), X(2),
                      X(3), X(4), X(5),
                      X(6), X(7), X(8);

    std::cout << "solve finish!!!!!!!!!!!" << std::endl;
    return correct_matrix;
}

// check if data is full
// if true, then do least square
bool OdomCalib::is_full()
{
    if(now_len % data_len == 0 && now_len >= 1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

// clean data to zero
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
