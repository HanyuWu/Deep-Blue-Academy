#include "map.h"
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值 (L) 
 * ans(1:2)表示梯度 (dL/dx ; dL/dy)
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    //TODO
    double x_true = coords(0);
    double y_true = coords(1);

    int x_floor = (int)x_true;
    int y_floor = (int)y_true;

    double x_prime = x_true - x_floor;     // 数值稳定性
    double y_prime = y_true - y_floor;

    double Z_00 = map->cells[MAP_INDEX(map, x_floor, y_floor)].score;
    double Z_10 = map->cells[MAP_INDEX(map, (x_floor+1), y_floor)].score;
    double Z_11 = map->cells[MAP_INDEX(map, (x_floor+1), (y_floor+1))].score;
    double Z_01 = map->cells[MAP_INDEX(map, x_floor, (y_floor+1))].score;
    
    double x_prime_minus = 1.0 - x_prime; 
    double y_prime_minus = 1.0 - y_prime;

    double L = y_prime*(x_prime*Z_11 + x_prime_minus*Z_01) + y_prime_minus*(x_prime*Z_10 + x_prime_minus*Z_00);
    double dl_dx = (y_prime*(Z_11-Z_01) + y_prime_minus*(Z_10-Z_00))/map->resolution;
    double dl_dy = (x_prime*Z_11 + x_prime_minus*Z_01 - x_prime* Z_10 - x_prime_minus*Z_00)/map->resolution;
    // Divide by resolution -> from map derivative to actual derivative


    ans << L, dl_dx, dl_dy;
    //END OF TODO

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO
    Eigen::Matrix3d mat_rotate;
    double c = cos(now_pose(2));
    double s = sin(now_pose(2));

    mat_rotate << c, -s, now_pose(0),
                  s, c,  now_pose(1),
                  0, 0,  1;

    for (auto pts: laser_pts){

        Eigen::Vector3d laser_pose;
        laser_pose << pts(0), pts(1), 1;
        laser_pose = mat_rotate * laser_pose;

        double cell_x = (laser_pose[0] - map->origin_x) / map->resolution + double(map->size_x / 2);    
        double cell_y = (laser_pose[1] - map->origin_y) / map->resolution + double(map->size_y / 2);
        // actual laser coord -> map cell coord

        Eigen::Vector2d S_coord(cell_x, cell_y);

        Eigen::Vector3d M = InterpMapValueWithDerivatives(map,S_coord);

        Eigen::Vector2d delta_M(M(1),M(2));

        Eigen::MatrixXd dS_dT(2,3);
        dS_dT << 1, 0, -s*pts(0)-c*pts(1),
                 0, 1, c*pts(0)-s*pts(1);

        Eigen::MatrixXd temp = delta_M.transpose()*dS_dT;
        H = H + temp.transpose()*temp;
        b = b + temp.transpose()*(1-M(0));
    }
    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    for(int i = 0; i < maxIteration;i++)
    {
        //TODO
        Eigen::Vector3d delta_pose;
        Eigen::Matrix3d H; 
        Eigen::Vector3d b;

        ComputeHessianAndb(map, now_pose, laser_pts, H, b);
        delta_pose = H.inverse()*b;
        now_pose = now_pose + delta_pose;
        //END OF TODO
    }
    init_pose = now_pose;

}
