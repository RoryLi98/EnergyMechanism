#include"energy.h"
//计算距离
float calculate_distance(Point2f a, Point2f b)
{
    return sqrt((a.x-b.x) * (a.x-b.x) + (a.y - b.y) * (a.y - b.y));
}

//比较距离
bool comp_distance(Perhaps_scores a, Perhaps_scores b)
{
    return a.distance < b.distance;
}

//画矩形：框出需要处理、击打的目标
void drawBox(RotatedRect box, Mat &img)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }
    box.points(pt); //计算二维盒子顶点  0右下角的点，顺时针，3右上角的点
    line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 2, 8, 0);
}

Energy_Agency::Energy_Agency()
{
    train_hog = new HOGDescriptor(Size(64,128), Size(16,16), Size(8,8), Size(8,8), 9);

//    predict_model = cv::ml::SVM::load("/home/teliute/Link/TODAY/hog_model_small.xml");//====================================
    predict_model = cv::ml::SVM::load("/home/link/EnergyNew/hog_model_small.xml");
    //*********************改svm路径*****************************
}

void Energy_Agency::EnergyProcess(Mat &src, Mat &mask_bright, Mat& mask_color,int ColorNumber)  //亮度&颜色，在main中被调用 1蓝2红
{
    Mat color_light, gray_image; //通道相减后图像,亮度二值化图像,颜色二值化图像;
    vector<Mat> bgr_channel;
    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
    Mat element1 = getStructuringElement(MORPH_ELLIPSE,Size(4,4));
    split(src, bgr_channel);
    cvtColor(src, gray_image, COLOR_BGR2GRAY);

    int energy_agency_color_thresh;
    int energy_agency_bright_thresh;
    if(ColorNumber==1)
    {
        subtract(bgr_channel[0], bgr_channel[2], color_light);
        energy_agency_color_thresh =65; //蓝65 红40
        energy_agency_bright_thresh =40; //蓝40 红25
    }
    if(ColorNumber==2)
    {
        subtract(bgr_channel[2], bgr_channel[0], color_light);
        energy_agency_color_thresh =40; //蓝65 红40
        energy_agency_bright_thresh =25; //蓝40 红25
    }

    threshold(color_light, mask_color, energy_agency_color_thresh, 255, CV_THRESH_BINARY);

//    medianBlur(mask_color,mask_color,3);
//    medianBlur(mask_color,mask_color,3);
//    medianBlur(mask_color,mask_color,3);
    medianBlur(mask_color,mask_color,5);

    dilate(mask_color, mask_color, element1, Point(-1, -1), 1);
    threshold(gray_image, mask_bright, energy_agency_bright_thresh, 255, CV_THRESH_BINARY);
    dilate(mask_bright, mask_bright, element, Point(-1,-1), 1);

    imshow("color_light",mask_color);
    imshow("bright_image", mask_bright);

//    mask_bright =  mask_bright;
    mask_bright = mask_color & mask_bright;
    imshow("remix_image", mask_bright);

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
            Energy_Agency_Rotated_Arm c;
            RotatedRect rect = minAreaRect(Energy_Agency_contours[i]);  //==============================
            float big_height = MIN(rect.size.height, rect.size.width);
            float big_weight = MAX(rect.size.height, rect.size.width);
            //cout<<"ccccsize:"<<rect.size.area()<<endl;
            if(rect.size.area() < THRESHOLD_CIRCLECENTER_BIGEST && rect.size.area()>THRESHOLD_CIRCLECENTER_SMALLEST )
            {//if(rect.size.area() < THRESHOLD_CIRCLECENTER_BIGEST && rect.size.area()>THRESHOLD_CIRCLECENTER_SMALLEST && big_height/big_weight > 0.5 && big_weight/big_height<1.5) //外轮廓且方
                circle_center.push_back(rect.center);
                //cout<<"add"<<endl;
            }
            if(rect.size.area() < 300 || big_height/big_weight > 0.8 || big_height/big_weight < 0.2)
            {   //cout<<"InEnergy_Agency_choose_perhaps_targets: rect.size.area out"<<endl;
                //if(rect.size.area() > 300)
                //    cout<<rect.size.area() <<" ; "<< big_height/big_weight <<" ; "<< big_height/big_weight <<endl;
                continue;
            }

            c.extern_rotatedrect = rect;

            int j=Energy_Agency_hierarchy[i][2];
            if(j==-1) continue;                    //该外轮廓无子轮廓，所以是无关轮廓


            while(j!=-1)
            {
                if(Energy_Agency_contours[j].size() > 10 )
                {
                    RotatedRect armor_rect = minAreaRect(Energy_Agency_contours[j]);
                    float height = MIN(armor_rect.size.height, armor_rect.size.width);
                    float width = MAX(armor_rect.size.height, armor_rect.size.width);

                    if(height / width > 0.4 && height / width < 0.8 )//
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
        return 0;
    }
    for(i = 0; i < rotated_arms.size(); i++)
    {
       RotatedRect c = rotated_arms[i].extern_rotatedrect;
       Point2f vertices[4], distance_points1, distance_points2;
       c.points(vertices);             //vertices[0]为最下方的点，若有两个最下方的点（四个边分别与x、y轴平行）则取左下方。顺时针vertices为最外大臂四点
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

EnergyResult Energy_Agency::Energy_Agency_choose_final_target(vector<Point2f> &circle_center, Mat &src, Mat &mask_bright, vector<Energy_Agency_Rotated_Arm> &rotated_arms, RotatedRect & final_armor,int &rotatedArmsIndex)
{
    int i;
    EnergyResult DetectResult;
    int result = Energy_Agency_target_filters(circle_center, mask_bright, rotated_arms);
    if(result == 0)
    {
        //cout<<"filters result=0"<<endl;
        DetectResult.flag = 0;
        return DetectResult;
    }

   // cout<<"送进SVM的个数 =："<<rotated_arms.size()<<endl;
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

       // cout<<"送进SVM的第i个的面积 =："<<rotated_arms[i].extern_rotatedrect.size.area()<<endl;

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

      //  imshow("svm_image",svm_image);

        int response = (int)predict_model->predict(line_i);

        //cout<<"response"<<response<<endl;
        if(response == 1)
        {
             drawBox(rotated_arms[i].perhaps_target,src);
            // drawBox(rotated_arms[i].extern_rotatedrect,src);  //======================================
             DetectResult.TargetRotatedRect =rotated_arms[i].perhaps_target;
             DetectResult.RotatedArmCenter =rotated_arms[i].extern_rotatedrect.center;
             DetectResult.flag = 1;
             final_armor = rotated_arms[i].perhaps_target;
             rotatedArmsIndex=i;
             return DetectResult;
        }
        //else{ cout<<"response==0; i= "<<i<<" ;rotated_arms.size() :"<<rotated_arms.size()<<endl; }
    }
    DetectResult.flag = 0;
    return DetectResult;
}

