#include "AngleSolver.h"
#define pi 3.141592653
using namespace std;
using namespace cv;

double predict_data(Mat A, double x){
  return A.at<double>(0,0) + A.at<double>(1,0) * x +
         A.at<double>(2,0) * pow(x, 2) + A.at<double>(3,0) * pow(x, 3);
}

double K[1][4] = {-12.71428571428515,0.05548941798940722,
                  -0.0001587301587301069,1.286008230451963e-07};
Mat A = Mat(4,1,CV_64FC1, K);

AngleSolver::AngleSolver(Mat cam_matrix, Mat distortion_coeff, double min_distance, double max_distance)
{
    _cam_matrix=cam_matrix.clone();
    _distortion_coeff=distortion_coeff.clone();
    _min_distance = min_distance;
    _max_distance=max_distance;
    _big_armor_height = 7.5;
    _big_armor_weight = 21.6;
    _small_armor_height = 7.2;
    _small_armor_weight = 11.5;
    _energy_armor_height = 7.5;
    _energy_armor_weight = 21.6;
    calculate_result = true;
}

void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans)
{
    if (_width_target < 10e-5 || _height_target < 10e-5) {
       rot = cv::Mat::eye(3, 3, CV_64FC1);
       trans = cv::Mat::zeros(3, 1, CV_64FC1);
       return;
    }
    std::vector<cv::Point3f> point3d;
    double half_x = _width_target / 2.0;
    double half_y = _height_target / 2.0;
    //点顺序为左上，左下，右下，右上
    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));
    cv::Mat r;
    cv::solvePnP(point3d, points2d, _cam_matrix, _distortion_coeff, r, trans);
    //罗格里斯转换
    Rodrigues(r, rot);
}

void AngleSolver::getTarget2dPoinstion(std::vector<Point2f> points, vector<Point2f> & target2d){
    Point2f lu, ld, ru, rd;
    sort(points.begin(), points.end(), [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });
    if (points[0].y < points[1].y){
        lu = points[0];
        ld = points[1];
    }
    else{
        lu = points[1];
        ld = points[0];
    }
    if (points[2].y < points[3].y){
        ru = points[2];
        rd = points[3];
    }
    else {
        ru = points[3];
        rd = points[2];
    }
    target2d.clear();
    target2d.push_back(lu);
    target2d.push_back(ru);
    target2d.push_back(rd);
    target2d.push_back(ld);
}

void AngleSolver::setRelationPoseCameraPTZ(const cv::Mat & rot_camera_y_ptz,
                                           const cv::Mat & rot_camera_x_ptz,
                                           const cv::Mat & trans_camera_ptz,
                                           double y_offset_barrel_ptz) {
    rot_camera_y_ptz.copyTo(_rot_y_camera2ptz);
    trans_camera_ptz.copyTo(_trans_camera2ptz);
    rot_camera_x_ptz.copyTo(_rot_x_camera2ptz);
    _offset_y_barrel_ptz = y_offset_barrel_ptz;
}

//能量机关anglesolve
void AngleSolver::setRelationPoseCameraPTZForEnergy(const cv::Mat & rot_camera_ptz,
                                                    const cv::Mat & trans_camera_ptz,
                                                    double y_offset_barrel_ptz){
    rot_camera_ptz.copyTo(_rot_camera2ptz);
    trans_camera_ptz.copyTo(_trans_camera2ptz);
    _offset_y_barrel_ptz = y_offset_barrel_ptz;
}

//能量机关为false
void AngleSolver::tranformationCamera2PTZ(const cv::Mat & pos,
                                          cv::Mat & transed_pos,
                                          bool is_attack_armor){
    //transed_pos = _rot_camera2ptz * _rot_x_camera2ptz * pos - _trans_camera2ptz;
    if(is_attack_armor){
        double predict_theta = predict_data(A, pos.at<double>(2,0));
        cout<<"predict:"<<predict_theta<<endl;
        double theta_y = 1 * pi/180;  //yaw 0
        double theta_x = predict_theta * pi/180; //pitch  3haobingbu -2
        double r_x_data[] = {1,0,0,0,cos(theta_x),sin(theta_x), 0, -sin(theta_x), cos(theta_x)};
        double r_y_data[] = {cos(theta_y), 0, -sin(theta_y), 0, 1, 0, sin(theta_y), 0, cos(theta_y)};
        double t_data[] = {0, -6, 0}; // ptz org position in camera coodinate system
        Mat t_camera_ptz(3,1, CV_64FC1, t_data);
        Mat ry_camera_ptz(3,3, CV_64FC1, r_y_data);//左右
        Mat rx_camera_ptz(3,3, CV_64FC1, r_x_data);//上下
        ry_camera_ptz.copyTo(_rot_y_camera2ptz);
        t_camera_ptz.copyTo(_trans_camera2ptz);
        rx_camera_ptz.copyTo(_rot_x_camera2ptz);
        transed_pos = _rot_y_camera2ptz * _rot_x_camera2ptz * (pos - _trans_camera2ptz);
    }
    else
        transed_pos = _rot_camera2ptz * pos - _trans_camera2ptz;
}

void AngleSolver::setTargetSize(TargetType type)
{
    if(type == TARGET_ARMOR){
        _width_target = _big_armor_weight;
        _height_target = _big_armor_height;
    }
    else if(type == TARGET_SAMLL_ARMOR){
        _width_target = _small_armor_weight;
        _height_target = _small_armor_height;
    }
    else if(type == TARGET_ENERGY){
        _width_target = _energy_armor_weight;
        _height_target = _energy_armor_height;
    }
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y){
    double alpha = 0.0, theta = 0.0;
    const double *xyz = (const double *)pos_in_ptz.data;
    alpha = asin(_offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    if(xyz[1] < 0){
        theta = atan(-xyz[1]/xyz[2]);
        angle_y = -(alpha+theta);  // camera coordinate
    }
    else if (xyz[1] < _offset_y_barrel_ptz){
        theta = atan(xyz[1]/xyz[2]);
        angle_y = -(alpha-theta);  // camera coordinate
    }
    else{
        theta = atan(xyz[1]/xyz[2]);
        angle_y = (theta-alpha);   // camera coordinate
    }
    angle_x = atan2(xyz[0], xyz[2]);
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}

bool AngleSolver::getAngle(vector<Point2f> & points, double & angle_x, double & angle_y,
                           TargetType type, bool isAttack_armor){
    setTargetSize(type);
    vector<Point2f> target2d;
    getTarget2dPoinstion(points, target2d);
    cv::Mat r;
    solvePnP4Points(target2d, r, _position_in_camera);
    if(isAttack_armor){
        _position_in_camera.at<double>(0,0) *= 0.92;
        _position_in_camera.at<double>(1,0) *= 0.92;
        _position_in_camera.at<double>(2,0) *= 0.92;
    }
    else{
        //===================================
//        cout<<"x:"<<_position_in_camera.at<double>(0, 0)<<endl;
//        cout<<"y:"<<_position_in_camera.at<double>(1, 0)<<endl;
        cout<<"distance:"<<_position_in_camera.at<double>(2, 0)<<"cm"<<endl;
        double ratioOfDistance=(754.98)/( _position_in_camera.at<double>(2, 0));
        _position_in_camera.at<double>(0, 0)*=ratioOfDistance;
        _position_in_camera.at<double>(1, 0)*=ratioOfDistance;
        _position_in_camera.at<double>(2, 0)*=ratioOfDistance;
    }

    if (_position_in_camera.at<double>(2, 0) < _min_distance || _position_in_camera.at<double>(2, 0) > _max_distance){
        calculate_result = false;
        return false;
    }
    tranformationCamera2PTZ(_position_in_camera, _position_in_ptz, isAttack_armor);
    adjustPTZ2Barrel(_position_in_ptz, angle_x, angle_y);
    calculate_result = true;
    return true;
}
