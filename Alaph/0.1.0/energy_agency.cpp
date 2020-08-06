#include "energy_agency.h"

//#define RUNE_BLUE
#define RUNE_RED

#define PROCESS_DEBUG "process_debug"

//#define SHOW_DEBUG //debug mode show process image

#define THRESHOLD_CIRCLECENTER_BIGEST 800   //480:600
#define THRESHOLD_CIRCLECENTER_SMALLEST 200

#define THRESHOLD_ROTATEDARM_BIGEST 5000   //480: 8000
#define THRESHOLD_ROTATEDARM_SMALLEST 1000

#define pi 3.141592653
//#define SHOW_PROCESS_IMAGE

//#define DEBUG_createTrackbar

using namespace std;
using namespace cv;
using namespace cv::ml;

void drawBox(RotatedRect box, Mat &img, Scalar theColor)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }

    box.points(pt); //计算二维盒子顶点  0右下角的点，顺时针，3右上角的点
    line(img, pt[0], pt[1], theColor, 2, 8, 0);
    line(img, pt[1], pt[2], theColor, 2, 8, 0);
    line(img, pt[2], pt[3], theColor, 2, 8, 0);
    line(img, pt[3], pt[0], theColor, 2, 8, 0);
}


Energy_Agency::Energy_Agency()
{
    train_hog = new HOGDescriptor(Size(64,128), Size(16,16), Size(8,8), Size(8,8), 9);
    energy_agency_color_thresh = 40;        //40
    energy_agency_bright_thresh = 40;       //40
}

void Energy_Agency::Energy_Agency_loadSVM_Model(string svm_Path)
{
    predict_model = SVM::load(svm_Path);
}

void Energy_Agency::set_roll_prevaricate(int roll_prevaricate)
{
    this->energy3dReal.energyAgencyRollDegree=16.32+roll_prevaricate;
}


void Energy_Agency::makeVideo_PushFrame(const Mat src)
{
    //VideoWriter vw("/home/rikirobot/Desktop/RM2019_Vision/1Record.avi", CV_FOURCC('M', 'J', 'P', 'G'), 60.0, Size(640, 480));
    if(makeTestVideoInfo.makeVideo==false)
    {
        return;
    }
    else if(makeTestVideoInfo.makeVideo==true)
    {
        if(src.empty())
        {
            //vw.release();
            return;
        }
        if(makeTestVideoInfo.energyAgencyVideoWriterCreatedAlready==false)
        {
            string makeVideoFileName=makeTestVideoInfo.saveVideoPath+"/energyAgencyVideo01"+".avi";
            makeTestVideoInfo.energyAgencyVideoWriter=VideoWriter(makeVideoFileName, CV_FOURCC('M', 'J', 'P', 'G'), 60.0, Size(1280, 720));

            if(makeTestVideoInfo.energyAgencyVideoWriter.isOpened()){ makeTestVideoInfo.energyAgencyVideoWriterCreatedAlready=true; }
            else { cout<<"In EnergyAgency makeTestVideo: can not created videoWriter! 可能没有读写存储器的权限! "<<endl; }
        }

        if(makeTestVideoInfo.energyAgencyVideoWriter.isOpened())
        {
            makeTestVideoInfo.energyAgencyVideoWriter << src;
            //imshow("src",src);
            waitKey(1);
        }
    }
}


//RotatedRect ->vector<Point2f>
void Energy_Agency::getRotatedRect4Points(RotatedRect InputRect,vector<Point2f>& get4Points)
{
    Point2f pointsArray[4];
    InputRect.points(pointsArray);

    get4Points.clear();
    for(int i=0;i<4;i++)
    {
        get4Points.push_back(pointsArray[i]);
    }

    Point2f lu, ld, ru, rd;
    //leftup leftdown rightup rightdown
    sort(get4Points.begin(), get4Points.end(), [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; }); //匿名函数定义的是从小到大
    if (get4Points[0].y < get4Points[1].y){
        lu = get4Points[0];
        ld = get4Points[1];
    }
    else{
        lu = get4Points[1];
        ld = get4Points[0];
    }
    if (get4Points[2].y < get4Points[3].y)	{
        ru = get4Points[2];
        rd = get4Points[3];
    }
    else {
        ru = get4Points[3];
        rd = get4Points[2];
    }

    get4Points.clear();
    get4Points.push_back(lu);
    get4Points.push_back(ru); //  0       1
    get4Points.push_back(rd); //  2       3
    get4Points.push_back(ld); // 左上开始顺时针

    //cout<<"In getRotatedRect4Points:"<<lu<<ru<<rd<<ld<<endl;
}

void Energy_Agency::create_debug_image()
{
    namedWindow(PROCESS_DEBUG);
    createTrackbar("color_thresh_bar", PROCESS_DEBUG, &energy_agency_color_thresh, 255, NULL);
    createTrackbar("bright_thresh_bar", PROCESS_DEBUG, &energy_agency_bright_thresh, 255, NULL);
}

void Energy_Agency::Energy_Agency_process(Mat &src, Mat &mask_bright, Mat& mask_color)  //亮度&颜色，在main中被调用
{
    Mat color_light, gray_image; //通道相减后图像,亮度二值化图像,颜色二值化图像;
    vector<Mat> bgr_channel;
    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
    split(src, bgr_channel);
    cvtColor(src, gray_image, COLOR_BGR2GRAY);

//Attention!!!!RUNE_RED:2\0;RUNE_BLUE:0\2(i changed it when i was debuging)
#ifdef RUNE_RED
    subtract(bgr_channel[0], bgr_channel[2], color_light);
#endif // RUNE_RED

#ifdef RUNE_BLUE
    subtract(bgr_channel[0], bgr_channel[2], color_light);
#endif // RUNE_BLUE

#ifdef DEBUG_createTrackbar
    energy_agency_color_thresh = getTrackbarPos("color_thresh", PROCESS_DEBUG);
    energy_agency_bright_thresh = getTrackbarPos("bright_thresh", PROCESS_DEBUG);
#endif
    energy_agency_color_thresh =65;
    //cout<<"energy_agency_color_thresh: "<<energy_agency_color_thresh<<"  energy_agency_bright_thresh: "<<energy_agency_bright_thresh<<endl;

    //imshow("Raw_color_light",color_light);
    //imshow("Raw_gray_image",gray_image);

    threshold(color_light, mask_color, energy_agency_color_thresh, 255, CV_THRESH_BINARY);

    medianBlur(mask_color,mask_color,3);
    medianBlur(mask_color,mask_color,3);
    medianBlur(mask_color,mask_color,3);
    medianBlur(mask_color,mask_color,5);
//    medianBlur(mask_color,mask_color,5);

//    imshow("codasdsadasdaght",mask_color);
    dilate(mask_color, mask_color, element, Point(-1, -1), 1);
    threshold(gray_image, mask_bright, energy_agency_bright_thresh, 255, CV_THRESH_BINARY);
    dilate(mask_bright, mask_bright, element, Point(-1,-1), 1);

    imshow("color_light",mask_color);
    imshow("bright_image", mask_bright);

    mask_bright = mask_color & mask_bright;


//    Mat element1 = getStructuringElement(MORPH_CROSS,Size(5,5));
//    Mat element2 = getStructuringElement(MORPH_RECT,Size(5,5));
//    erode(mask_bright, mask_bright,element2);
//    imshow("erode", mask_bright);
//    dilate(mask_bright, mask_bright, element2);

    imshow("remix_image", mask_bright);
#ifdef SHOW_PROCESS_IMAGE
    //imshow(PROCESS_DEBUG,mask_bright);
#endif

}

vector<Energy_Agency_Rotated_Arm> Energy_Agency::Energy_Agency_choose_perhaps_targets(vector<Point2f> &circle_center, Mat &mask_bright)
{
    vector<Energy_Agency_Rotated_Arm> Rotated_Arms;
    vector<vector<Point>> Energy_Agency_contours;
    vector<Vec4i> Energy_Agency_hierarchy;
    findContours(mask_bright, Energy_Agency_contours, Energy_Agency_hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    if (Energy_Agency_contours.size() == 0) {cout<<"contours errored!!"<<endl;  return Rotated_Arms;}
    int i;
    for(i = 0; i < Energy_Agency_contours.size(); i++)
    {
        //外轮廓
        if(Energy_Agency_contours[i].size() > 10 && Energy_Agency_hierarchy[i][3] == -1)
        {

//            RNG rng(12345);
//            // Draw contours
//            Mat drawing = Mat::zeros( mask_bright.size(), CV_8UC3 );
//            vector<vector<Point>> test_contours;
//           // test_contours.push_back(Energy_Agency_contours[i]);
//            for( size_t num = 0; num< Energy_Agency_contours.size(); num++ )
//            {
//                Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//                drawContours( drawing, Energy_Agency_contours, -1, color, 1, LINE_8, Energy_Agency_hierarchy, 0 );
//            }

//            /// Show in a window
//            imshow( "Contours", drawing );
//            waitKey(0);

            Energy_Agency_Rotated_Arm c;
            RotatedRect rect = minAreaRect(Energy_Agency_contours[i]);  //==============================
            float big_height = MIN(rect.size.height, rect.size.width);
            float big_weight = MAX(rect.size.height, rect.size.width);

            if(rect.size.area() < THRESHOLD_CIRCLECENTER_BIGEST && rect.size.area()>THRESHOLD_CIRCLECENTER_SMALLEST )
            //if(rect.size.area() < THRESHOLD_CIRCLECENTER_BIGEST && rect.size.area()>THRESHOLD_CIRCLECENTER_SMALLEST && big_height/big_weight > 0.5 && big_weight/big_height<1.5) //外轮廓且方
                circle_center.push_back(rect.center);


            if(rect.size.area() < 300 || big_height/big_weight > 0.8 || big_height/big_weight < 0.2)
            {   //cout<<"InEnergy_Agency_choose_perhaps_targets: rect.size.area out"<<endl;
                //if(rect.size.area() > 300)
                //    cout<<rect.size.area() <<" ; "<< big_height/big_weight <<" ; "<< big_height/big_weight <<endl;
                continue;}
            c.extern_rotatedrect = rect;

            int j=Energy_Agency_hierarchy[i][2];
            if(j==-1) continue; //这个外轮廓甚至连子轮廓都没有，所以是个酱油无关轮廓
            while(j!=-1)
            {
                if(Energy_Agency_contours[j].size() > 10 )
                {
                    RotatedRect armor_rect = minAreaRect(Energy_Agency_contours[j]);
                    float height = MIN(armor_rect.size.height, armor_rect.size.width);
                    float width = MAX(armor_rect.size.height, armor_rect.size.width);
                    if(height / width > 0.4 && height / width < 0.8 )
                    {
                        c.inside_rotatedrects.push_back(armor_rect);
                    }
                }
                j=Energy_Agency_hierarchy[j][0];//同级的下一个轮廓
            }

            if(c.inside_rotatedrects.size() > 0)
                Rotated_Arms.push_back(c);
            else
            {
                //cout<<"In Energy_Agency_choose_perhaps_targets: inside_rects.size=0"<<endl;
            }
        }

        //i=Energy_Agency_hierarchy[i][0];
    }

    return Rotated_Arms;
}

int Energy_Agency::Energy_Agency_target_filters(vector<Point2f> &circle_center, Mat &mask_bright, vector<Energy_Agency_Rotated_Arm> &rotated_arms)
{
    rotated_arms = Energy_Agency_choose_perhaps_targets(circle_center, mask_bright);    //最外大臂集

    int i,j;
    if(rotated_arms.size() == 0)
    {   //cout<<"Energy_Agency_target_filters: rotated_arms.size()=0"<<endl;
        return 0;}
    for(i = 0; i < rotated_arms.size(); i++)
    {
       RotatedRect c = rotated_arms[i].extern_rotatedrect;
       Point2f vertices[4], distance_points1, distance_points2;
       c.points(vertices); //vertices[0]为最下方的点，若有两个最下方的点（四个边分别与x、y轴平行）则取左下方。顺时针vertices为最外大臂四点
       vector<Perhaps_scores> scores;
       //求中心点到两边的距离，取最小的一个
       if(c.size.width > c.size.height)        //右上方的边为height：顺时针旋转，碰到的第一条边长，叫做height
       {
           distance_points1.x = (vertices[2].x + vertices[3].x)/2;
           distance_points1.y = (vertices[2].y + vertices[3].y)/2;
           distance_points2.x = (vertices[0].x + vertices[1].x)/2;
           distance_points2.y = (vertices[0].y + vertices[1].y)/2;

           for(j = 0; j < rotated_arms[i].inside_rotatedrects.size(); j++)
           {
               double distance_1 = calculate_distance(distance_points1, rotated_arms[i].inside_rotatedrects[j].center);
               double distance_2 = calculate_distance(distance_points2, rotated_arms[i].inside_rotatedrects[j].center);
               //cout<<distance_1<<" "<<distance_2<<endl;
               double min_distance = MIN(distance_1, distance_2);
               Perhaps_scores score;
               score.distance = min_distance;
               score.target = rotated_arms[i].inside_rotatedrects[j];
               if(min_distance == distance_1)
               {
                    score.sort_vertices.push_back(vertices[2]);
                    score.sort_vertices.push_back(vertices[3]);
                    score.sort_vertices.push_back(vertices[0]);
                    score.sort_vertices.push_back(vertices[1]);
               }
               else if(min_distance == distance_2)
               {
                   score.sort_vertices.push_back(vertices[0]);
                   score.sort_vertices.push_back(vertices[1]);
                   score.sort_vertices.push_back(vertices[2]);
                   score.sort_vertices.push_back(vertices[3]);
               }
               if( score.target.size.area()>((1.0/20)*(rotated_arms[i].extern_rotatedrect.size.area() ) ) )    //筛去小干扰点190708
                    scores.push_back(score);
           }
           if(scores.size()<=0)
               cout<<"scoresSize==0!!"<<endl;

           if(scores.size()>0)
           {
               sort(scores.begin(), scores.end(), comp_distance);
               rotated_arms[i].perhaps_vertices = scores[0].sort_vertices;  //
               rotated_arms[i].perhaps_target = scores[0].target;           //打击点
           }

       }
       else if(c.size.width < c.size.height)
       {
           distance_points1.x = (vertices[1].x + vertices[2].x)/2;
           distance_points1.y = (vertices[1].y + vertices[2].y)/2;
           distance_points2.x = (vertices[0].x + vertices[3].x)/2;
           distance_points2.y = (vertices[0].y + vertices[3].y)/2;

           for(j = 0; j < rotated_arms[i].inside_rotatedrects.size(); j++)
           {
               double distance_1 = calculate_distance(distance_points1, rotated_arms[i].inside_rotatedrects[j].center);
               double distance_2 = calculate_distance(distance_points2, rotated_arms[i].inside_rotatedrects[j].center);
               double min_distance = MIN(distance_1, distance_2);
               Perhaps_scores score;
               score.distance = min_distance;
               score.target = rotated_arms[i].inside_rotatedrects[j];
               if(min_distance == distance_1)
               {
                    score.sort_vertices.push_back(vertices[1]);
                    score.sort_vertices.push_back(vertices[2]);
                    score.sort_vertices.push_back(vertices[3]);
                    score.sort_vertices.push_back(vertices[0]);
               }
               else if(min_distance == distance_2)
               {
                   score.sort_vertices.push_back(vertices[3]);
                   score.sort_vertices.push_back(vertices[0]);
                   score.sort_vertices.push_back(vertices[1]);
                   score.sort_vertices.push_back(vertices[2]);
               }
               if( score.target.size.area()>((1.0/20)*(rotated_arms[i].extern_rotatedrect.size.area() ) ) )    //筛去小干扰点190708
                    scores.push_back(score);
           }
            if(scores.size()<=0) cout<<"scoresSize==0!!"<<endl;

           if(scores.size()>0)
           {
               sort(scores.begin(), scores.end(), comp_distance);
               rotated_arms[i].perhaps_vertices = scores[0].sort_vertices;  //
               rotated_arms[i].perhaps_target = scores[0].target;           //打击点
           }
       }
    }

    return 1;
}

vector<Point2f> Energy_Agency::Energy_Agency_choose_final_target(vector<Point2f> &circle_center, Mat &src, Mat &mask_bright, vector<Energy_Agency_Rotated_Arm> &rotated_arms, RotatedRect & final_armor,int &rotatedArmsIndex)
{
    int i;
    vector<Point2f> detect_result;
    int result = Energy_Agency_target_filters(circle_center, mask_bright, rotated_arms);
    if(result == 0)
    {    //cout<<"filters result=0"<<endl;
        return detect_result;}
    //cout<<"rotated_arms.size() =："<<rotated_arms.size()<<endl;
    for(i = 0; i < rotated_arms.size(); i++)
    {
        if( rotated_arms[i].perhaps_vertices.size()==0 )
        {
            //cout<<"noShootPoint"<<endl;
            continue;
        }  //这里筛出的原因主要是因为没有找到合适的打击点（190708）因为前面加了面积筛出，所以有可能perhaps_vertices perhaps_target直接就是空的

        //面积筛除
        //if(rotated_arms[i].extern_rotatedrect.size.area()>THRESHOLD_ROTATEDARM_BIGEST ) continue;
        if(rotated_arms[i].extern_rotatedrect.size.area()<THRESHOLD_ROTATEDARM_SMALLEST) continue;

        Mat svm_image = Mat::zeros(128, 64, CV_8UC3);
        vector<Point2f> dst_pt;
//        dst_pt.push_back(Point2f(0,0));
//        dst_pt.push_back(Point2f(svm_image.cols,0));
//        dst_pt.push_back(Point2f(svm_image.cols,svm_image.rows));
//        dst_pt.push_back(Point2f(0,svm_image.rows));
        dst_pt.push_back(Point2f(5,5));
        dst_pt.push_back(Point2f(svm_image.cols-5,5));
        dst_pt.push_back(Point2f(svm_image.cols-5,svm_image.rows-5));
        dst_pt.push_back(Point2f(5,svm_image.rows-5));

        Mat transmtx = getPerspectiveTransform(rotated_arms[i].perhaps_vertices, dst_pt);//透视变换
        warpPerspective(src, svm_image, transmtx, svm_image.size());

        resize(svm_image, svm_image, Size(64,128));
        vector<float> descripors;
        train_hog->compute(svm_image, descripors, Size(8,8));
        //cout<<"response.size"<<descripors.size()<<endl;
        Mat line_i = Mat(descripors).reshape(1,1);

        imshow("svm_image",svm_image);

        int response = (int)predict_model->predict(line_i);

        //cout<<"response"<<response<<endl;
        if(response == 1)
        {
             drawBox(rotated_arms[i].perhaps_target,src);

             drawBox(rotated_arms[i].extern_rotatedrect,src,Scalar(255,0,0));  //======================================

             detect_result.push_back(rotated_arms[i].perhaps_target.center);
             detect_result.push_back(rotated_arms[i].extern_rotatedrect.center);
             final_armor = rotated_arms[i].perhaps_target;
             rotatedArmsIndex=i;
             return detect_result;
        }
        //else{ cout<<"response==0; i= "<<i<<" ;rotated_arms.size() :"<<rotated_arms.size()<<endl; }
    }
    return detect_result;
}

//寻找最匹配的圆心值
Point2f Energy_Agency::calculate_probably_circle_center(vector<Point2f> armor_center, vector<Point2f> match_centers,RotatedRect final_armor)
{
    //armor_center[0]=perhaps_target.center ;    装甲板矩阵
    //armor_center[1]=extern_rotatedrect.center; 扇叶矩阵
    //match_centers 是所有外轮廓的中心点集合

    //构造直线y = kx + b;
    double k, b;
    vector<sort_center> sort_centers;
    Point2f circle_center = Point2f(-1,-1);
    if(armor_center[0].x - armor_center[1].x == 0)
    {
        k = armor_center[0].x;
        b = 0;
    }
    else
    {
        k = (armor_center[1].y - armor_center[0].y)/(armor_center[1].x - armor_center[0].x);
        b = armor_center[1].y - k * armor_center[1].x;
    }

    //求点到直线最小值,并且为靠近旋转臂距离端

    for(int i = 0; i < match_centers.size(); i++)
    {
        //筛选掉靠旋转臂中心距离大于装甲板中心距离的点
        if(calculate_distance(armor_center[0], match_centers[i]) < 1.3 * calculate_distance(armor_center[1], match_centers[i])||
           calculate_distance(armor_center[0], match_centers[i]) > 2.5 * calculate_distance(armor_center[1], match_centers[i])||
           calculate_distance(armor_center[0], match_centers[i]) < 3 * MIN(final_armor.size.width,final_armor.size.height)    )
        continue;

        //求点到直线距离,存入vector
        sort_center c;
        c.center = match_centers[i];
        if(armor_center[0].x - armor_center[1].x == 0)
        {
            c.distance = calculate_dot_to_line_distance(match_centers[i], k, b, 1);
        }
        else
        {
            c.distance = calculate_dot_to_line_distance(match_centers[i], k, b, 0);
        }
        sort_centers.push_back(c);
    }

    //cout<<"sortCenter: "<<sort_centers.size()<<endl;

    if(sort_centers.size() > 0)
    {
        sort(sort_centers.begin(), sort_centers.end(), comp_dot_distance);
        for(int i = 0; i < sort_centers.size(); i++)
        {
            if(sort_centers[i].distance < (1.0/2)*(MAX(final_armor.size.width,final_armor.size.height) ) )
            {
                return sort_centers[0].center;
            }
        }
    }
    return circle_center;
}



int Energy_Agency::judgeTheClockWise()
{
    //ClockWise         1
    //Anti-ClockWise    0
    //can not judge: guess from history,but dont change history
    //if can not judge and history.size()==0 ,guess wise one time

    int it=( this->energyHistory.previousTargetInRSystemAtan.size() -1 );
    int clockWise=1;//judge ClockWise

    if(this->energyHistory.previousTargetInRSystemAtan.size()>2 && this->energyHistory.previousTargetInRSystem.size()==this->energyHistory.previousTargetInRSystemAtan.size() )// 能否判断条件1
    {
        double diffOfAtan=this->energyHistory.previousTargetInRSystemAtan[it]-this->energyHistory.previousTargetInRSystemAtan[it-1];
        if( diffOfAtan>(-10) && diffOfAtan<10 && diffOfAtan!=0 )//若以60帧且每帧都检测到计算，事实上每次最多转1度 其实可以设为72，因为并不用担心打击后换新打击板的对称问题，在下面会筛去
        {
            //cout<<"diffOfAtan : "<<diffOfAtan<<endl;
            if( ((this->energyHistory.previousTargetInRSystem[it].x)*(this->energyHistory.previousTargetInRSystem[it-1].x))>0 ) //在y轴同一边
            {
                //所有情况符合条件，开始计算：
                if( diffOfAtan<0 ) //顺时针
                {
                    clockWise=1;
                }
                else if(diffOfAtan>0)   //逆时针
                {
                    clockWise=0;
                }
                this->energyHistory.judgeClockWiseHistory.push_back(clockWise);

                //cout<<"test: diffOfAtan:   "<<diffOfAtan<<endl;

                if(this->energyHistory.judgeClockWiseHistory.size()>=20)//存10个记录即可，多余的删掉吧
                {
                    this->energyHistory.judgeClockWiseHistory.erase(std::begin(this->energyHistory.judgeClockWiseHistory));// erase the front
                }
            }
        }
    }

    //can not judge debug      ==========================
//    if(this->energyHistory.previousTargetInRSystem.size()!=this->energyHistory.previousTargetInRSystemAtan.size()) cout<<"judgeTheClockWise ERROR: previousPoint.size()!=previousAtan.size()"<<endl;
//    else if(this->energyHistory.previousTargetInRSystemAtan.size()<2) cout<<"judgeTheClockWise WARMING: previousPoint.size()<2"<<endl;
//    else if( ((this->energyHistory.previousTargetInRSystem[it].x)*(this->energyHistory.previousTargetInRSystem[it-1].x))<0  )
//        {cout<<"judgeTheClockWise : not in same side from y ;pre,cur:"<<energyHistory.previousTargetInRSystem[it-1]<<energyHistory.previousTargetInRSystem[it]<<endl;}
//    else if(this->energyHistory.previousTargetInRSystemAtan[it]-this->energyHistory.previousTargetInRSystemAtan[it-1]<(-10) || this->energyHistory.previousTargetInRSystemAtan[it]-this->energyHistory.previousTargetInRSystemAtan[it-1]>10)
//        {cout<<"judgeTheClockWise : previousAtan change too quick,may be new one , change: "<<this->energyHistory.previousTargetInRSystemAtan[it]-this->energyHistory.previousTargetInRSystemAtan[it-1]<<endl;}
//    else if(this->energyHistory.previousTargetInRSystemAtan[it]-this->energyHistory.previousTargetInRSystemAtan[it-1]==0) cout<<"judgeTheClockWise : previousAtan didnt change"<<endl;


    //统计历史以判断顺逆：
    int sayWise=0;     //顺时针
    int sayAntiWise=0;  //逆时针

    //cout<<"clockWiseHistory: ";
    for(int i=0;i<this->energyHistory.judgeClockWiseHistory.size();i++)
    {
        if( this->energyHistory.judgeClockWiseHistory[i]==1 ) sayWise++;
        else if(this->energyHistory.judgeClockWiseHistory[i]==0 ) sayAntiWise++;

        //cout<<energyHistory.judgeClockWiseHistory[i];
    }
    //cout<<endl;

    if(sayAntiWise>sayWise)
    {
        return 0;   //逆时针
    }
    else if((sayAntiWise<sayWise))
    {
        return 1;   //顺时针
    }
    else if(this->energyHistory.judgeClockWiseHistory.size()>=1 && this->energyHistory.judgeClockWiseHistory[(energyHistory.judgeClockWiseHistory.size()-1)]==0)
    {
        return 0;
    }
    else if(this->energyHistory.judgeClockWiseHistory.size()>=1 && this->energyHistory.judgeClockWiseHistory[(energyHistory.judgeClockWiseHistory.size()-1)]==1)
    {
        return 1;
    }
    else
    {
        return 1;
    }
}

void Energy_Agency::deleteClockWise1History()
{
    if(this->energyHistory.judgeClockWiseHistory.size()>0)
    {
        this->energyHistory.judgeClockWiseHistory.erase(std::begin(this->energyHistory.judgeClockWiseHistory));
        // erase the front
    }
}

void Energy_Agency::refreshStructInfo(Point2f targetCenterImg_notD, Point2f& circleCenter_notD, Point2f rotatedArmCenter,RotatedRect final_armor)
{
//    this->energy3dReal;
//    this->energy2dImgD;
    //转为笛卡尔坐标系:
    this->energy2dImgD.targetCenterD=Point2f( targetCenterImg_notD.x , ( 720-targetCenterImg_notD.y ) );
    if(circleCenter_notD.x>0 && circleCenter_notD.y>0 && circleCenter_notD.x<1280 && circleCenter_notD.y<720 )
    {
        //this->energy2dImgD.circleCenterD=Point2f( circleCenter_notD.x , ( 480-circleCenter_notD.y ) );

        //因为有时候会圈圈到臂上的箭头，所以我决定放弃直接使用搜到的圆心
        //-------------------0730--------------------
        double mindx,mindy;
        mindx=fabs(final_armor.center.x-circleCenter_notD.x);
        mindy=fabs(final_armor.center.y-circleCenter_notD.y);


        double mindxdy=sqrt(mindx*mindx+mindy*mindy);
        double dxPointLen=this->energy2dReal.radiusInReal_PIX*mindx/mindxdy;
        double dyPointLen=this->energy2dReal.radiusInReal_PIX*mindy/mindxdy;


        double guessCircleCenterX,guessCircleCenterY;

        if((rotatedArmCenter.y-final_armor.center.y)!=0)
        {
            guessCircleCenterX=( final_armor.center.x + ((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))*dxPointLen  );
            guessCircleCenterY=( final_armor.center.y + ((rotatedArmCenter.y-final_armor.center.y)/fabs(rotatedArmCenter.y-final_armor.center.y))*dyPointLen  );
        }
        else
        {
            guessCircleCenterX=( final_armor.center.x + ((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))*dxPointLen  );
            guessCircleCenterY=( final_armor.center.y + 0*dyPointLen  );
        }

        circleCenter_notD=Point2f( guessCircleCenterX , guessCircleCenterY );
        this->energy2dImgD.circleCenterD=Point2f( circleCenter_notD.x , ( 720-circleCenter_notD.y ) );
        //cout<<"CircleCenter(in function mark1)="<<circleCenter_notD<<endl;   //============================

    }
    else
    {    //this->energy2dImgD.circleCenterD=Point2f( rotatedArmCenter.x , ( 480-rotatedArmCenter.y ) );
        Point2f final_armor4Points[4];
        final_armor.points(final_armor4Points);

        double minDistance=100000;
        int minDistanceIndex=0;
        double mindx,mindy;
        for(int i=0;i<4;i++)
        {
            double distanceBetweenPoints = calculate_distance(final_armor4Points[i],final_armor4Points[(i+1)%4]);
            if(distanceBetweenPoints<minDistance)
            {
                minDistance=distanceBetweenPoints;
                minDistanceIndex=i;
            }
        }
        mindx=fabs(final_armor4Points[minDistanceIndex].x-final_armor4Points[(minDistanceIndex+1)%4].x);
        mindy=fabs(final_armor4Points[minDistanceIndex].y-final_armor4Points[(minDistanceIndex+1)%4].y);

        double mindxdy=sqrt(mindx*mindx+mindy*mindy);
        double dxPointLen=this->energy2dReal.radiusInReal_PIX*mindx/mindxdy;
        double dyPointLen=this->energy2dReal.radiusInReal_PIX*mindy/mindxdy;

//        cout<<"dxPointLen: "<<dxPointLen<<endl;  ====================
//        cout<<"dyPointLen: "<<dyPointLen<<endl;
        //cout<<"mindx: "<<mindx<<" mindy: "<<mindy<<endl;
        //cout<<((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))<<"((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))"<<endl;
        //cout<<((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))<<"((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))"<<endl;
        //cout<<"index: "<<minDistanceIndex<<endl;

        double guessCircleCenterX,guessCircleCenterY;

        //用小装甲板长宽
        //guessCircleCenterX=( final_armor.center.x +  5.3*((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))*mindx  );
        //guessCircleCenterY=( final_armor.center.y +  5.3*((rotatedArmCenter.y-final_armor.center.y)/fabs(rotatedArmCenter.y-final_armor.center.y))*mindy  );

        //直接用struct中的rad
        if((rotatedArmCenter.y-final_armor.center.y)!=0)
        {
            guessCircleCenterX=( final_armor.center.x + ((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))*dxPointLen  );
            guessCircleCenterY=( final_armor.center.y + ((rotatedArmCenter.y-final_armor.center.y)/fabs(rotatedArmCenter.y-final_armor.center.y))*dyPointLen  );
        }
        else
        {
            guessCircleCenterX=( final_armor.center.x + ((rotatedArmCenter.x-final_armor.center.x)/fabs(rotatedArmCenter.x-final_armor.center.x))*dxPointLen  );
            guessCircleCenterY=( final_armor.center.y + 0*dyPointLen  );
        }

        circleCenter_notD=Point2f( guessCircleCenterX , guessCircleCenterY );
        this->energy2dImgD.circleCenterD=Point2f( circleCenter_notD.x , ( 720-circleCenter_notD.y ) );
        //cout<<"CircleCenter(in function mark2)="<<circleCenter_notD<<endl;   //====================================

        //cout<<"guessCircleCenterX: "<<guessCircleCenterX<<"guessCircleCenterY: "<<guessCircleCenterY<<endl;

    }

    this->energy2dImgD.targetInRSystem=this->energy2dImgD.targetCenterD-this->energy2dImgD.circleCenterD;
    //this->energy2dImgD.targetInRSystemAtan=( atan( ((this->energy2dImgD.targetInRSystem.y))/(this->energy2dImgD.targetInRSystem.x) ) )*( 180/pi );
    this->energy2dReal.targetInRSystemAtan=( atan( ((this->energy2dImgD.targetInRSystem.y)/0.987000)/(this->energy2dImgD.targetInRSystem.x) ) )*( 180/pi );

//    cout<<"targetInRSystem     : "<< this->energy2dImgD.targetInRSystem <<endl;
//    cout<<"targetInRSystemAtan : "<< (this->energy2dImgD.targetInRSystemAtan) <<endl;

    this->energyHistory.previousTargetInRSystem.push_back(this->energy2dImgD.targetInRSystem);
    this->energyHistory.previousTargetInRSystemAtan.push_back(this->energy2dReal.targetInRSystemAtan);
    if(this->energyHistory.previousTargetInRSystem.size()>3)
    {
        this->energyHistory.previousTargetInRSystem.erase(std::begin(this->energyHistory.previousTargetInRSystem));// erase the front
    }
    if(this->energyHistory.previousTargetInRSystemAtan.size()>3)
    {
        this->energyHistory.previousTargetInRSystemAtan.erase(std::begin(this->energyHistory.previousTargetInRSystemAtan));
    }


}


void Energy_Agency::shootRollEnergyAgency(cv::Point2f &forecastSeePoint)
{
    //=========judgeClockWise======
    int clockWise=this->judgeTheClockWise();
    Point2f forecastInRSystem;
    if(clockWise==0)
    {   //0.987000
        cout<<"Not clockWise"<<endl;
        forecastInRSystem.x=this->energy2dImgD.targetInRSystem.x*cos(this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180)
                -this->energy2dImgD.targetInRSystem.y/(0.987000)*sin(this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180);
        forecastInRSystem.y=this->energy2dImgD.targetInRSystem.x*sin(this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180)
                +this->energy2dImgD.targetInRSystem.y/(0.987000)*cos(this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180);
        forecastInRSystem.y=forecastInRSystem.y*(0.987000);
    }
    else
    {
        cout<<"ClockWise"<<endl;
        forecastInRSystem.x=this->energy2dImgD.targetInRSystem.x*cos(-this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180)
                -this->energy2dImgD.targetInRSystem.y/(0.987000)*sin(-this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180);
        forecastInRSystem.y=this->energy2dImgD.targetInRSystem.x*sin(-this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180)
                +this->energy2dImgD.targetInRSystem.y/(0.987000)*cos(-this->energy3dReal.energyAgencyRollDegree*this->energy3dReal.PI/180);
        forecastInRSystem.y=forecastInRSystem.y*(0.987000);
    }
    Point2f forecastD=Point2f( ( forecastInRSystem.x+this->energy2dImgD.circleCenterD.x ),( forecastInRSystem.y+this->energy2dImgD.circleCenterD.y ) );
    forecastSeePoint=Point2f( forecastD.x , (720-forecastD.y)  );
    cout<<"forecastSeePoint="<<forecastSeePoint<<endl;
}

void Energy_Agency::shootRollEnergyAgencyDebug(cv::Point2f &forecastSeePoint,double angle)
{
    int clockWise=this->judgeTheClockWise();
    Point2f forecastInRSystem;
    angle=fabs(angle);

    if(clockWise==0)
    {
        //cout<<"is not clockWise"<<endl;
        forecastInRSystem.x=this->energy2dImgD.targetInRSystem.x*cos(angle*this->energy3dReal.PI/180)
                -this->energy2dImgD.targetInRSystem.y/(0.987000)*sin(angle*this->energy3dReal.PI/180);
        forecastInRSystem.y=this->energy2dImgD.targetInRSystem.x*sin(angle*this->energy3dReal.PI/180)
                +this->energy2dImgD.targetInRSystem.y/(0.987000)*cos(angle*this->energy3dReal.PI/180);
        forecastInRSystem.y=forecastInRSystem.y*(0.987000);
    }
    else
    {
        //cout<<"is ClockWise"<<endl;
        forecastInRSystem.x=this->energy2dImgD.targetInRSystem.x*cos(-angle*this->energy3dReal.PI/180)
                -this->energy2dImgD.targetInRSystem.y/(0.987000)*sin(-angle*this->energy3dReal.PI/180);
        forecastInRSystem.y=this->energy2dImgD.targetInRSystem.x*sin(-angle*this->energy3dReal.PI/180)
                +this->energy2dImgD.targetInRSystem.y/(0.987000)*cos(-angle*this->energy3dReal.PI/180);
        forecastInRSystem.y=forecastInRSystem.y*(0.987000);
    }

    Point2f forecastD=Point2f( ( forecastInRSystem.x+this->energy2dImgD.circleCenterD.x ),( forecastInRSystem.y+this->energy2dImgD.circleCenterD.y ) );
    forecastSeePoint=Point2f( forecastD.x , (720-forecastD.y)  );
}

void Energy_Agency::translateRect(Point2f beforeCenter,Point2f afterCenter,vector<Point2f> &IORect4Points )
{
    double dx=afterCenter.x-beforeCenter.x;
    double dy=afterCenter.y-beforeCenter.y;

    for(int i=0;i<4;i++)
    {
        IORect4Points[i].x=IORect4Points[i].x+dx;
        IORect4Points[i].y=IORect4Points[i].y+dy;
    }
}
