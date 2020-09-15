#include "linefeature_tracker.h"
// #include "line_descriptor/src/precomp_custom.hpp"

LineFeatureTracker::LineFeatureTracker()
{
    allfeature_cnt = 0;
    frame_cnt = 0;
    sum_time = 0.0;
}

void LineFeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    

}

vector<Line> LineFeatureTracker::undistortedLineEndPoints()
{
    vector<Line> un_lines;
    un_lines = curframe_->vecLine;
    float fx = K_.at<float>(0, 0);
    float fy = K_.at<float>(1, 1);
    float cx = K_.at<float>(0, 2);
    float cy = K_.at<float>(1, 2);
    for (unsigned int i = 0; i <curframe_->vecLine.size(); i++)
    {
        un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx)/fx;
        un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy)/fy;
        un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx)/fx;
        un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy)/fy;
    }
    return un_lines;
}

void LineFeatureTracker::NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines,
                                            vector<pair<int, int> > &lineMatches) {

    float th = 3.1415926/9;
    float dth = 30 * 30;
    for (size_t i = 0; i < forw_lines.size(); ++i) {
        Line lf = forw_lines.at(i);
        Line best_match;
        size_t best_j = 100000;
        size_t best_i = 100000;
        float grad_err_min_j = 100000;
        float grad_err_min_i = 100000;
        vector<Line> candidate;

        // 从 forw --> cur 查找
        for(size_t j = 0; j < cur_lines.size(); ++j) {
            Line lc = cur_lines.at(j);
            // condition 1
            Point2f d = lf.Center - lc.Center;
            float dist = d.dot(d);
            if( dist > dth) continue;  //
            // condition 2
            float delta_theta1 = fabs(lf.theta - lc.theta);
            float delta_theta2 = 3.1415926 - delta_theta1;
            if( delta_theta1 < th || delta_theta2 < th)
            {
                //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                candidate.push_back(lc);
                //float cost = fabs(lf.image_dx - lc.image_dx) + fabs( lf.image_dy - lc.image_dy) + 0.1 * dist;
                float cost = fabs(lf.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                //std::cout<< "line match cost: "<< cost <<" "<< cost - sqrt( dist )<<" "<< sqrt( dist ) <<"\n\n";
                if(cost < grad_err_min_j)
                {
                    best_match = lc;
                    grad_err_min_j = cost;
                    best_j = j;
                }
            }

        }
        if(grad_err_min_j > 50) continue;  // 没找到

        //std::cout<< "!!!!!!!!! minimal cost: "<<grad_err_min_j <<"\n\n";

        // 如果 forw --> cur 找到了 best, 那我们反过来再验证下
        if(best_j < cur_lines.size())
        {
            // 反过来，从 cur --> forw 查找
            Line lc = cur_lines.at(best_j);
            for (int k = 0; k < forw_lines.size(); ++k)
            {
                Line lk = forw_lines.at(k);

                // condition 1
                Point2f d = lk.Center - lc.Center;
                float dist = d.dot(d);
                if( dist > dth) continue;  //
                // condition 2
                float delta_theta1 = fabs(lk.theta - lc.theta);
                float delta_theta2 = 3.1415926 - delta_theta1;
                if( delta_theta1 < th || delta_theta2 < th)
                {
                    //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                    //candidate.push_back(lk);
                    //float cost = fabs(lk.image_dx - lc.image_dx) + fabs( lk.image_dy - lc.image_dy) + dist;
                    float cost = fabs(lk.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                    if(cost < grad_err_min_i)
                    {
                        grad_err_min_i = cost;
                        best_i = k;
                    }
                }

            }
        }

        if( grad_err_min_i < 50 && best_i == i){

            //std::cout<< "line match cost: "<<grad_err_min_j<<" "<<grad_err_min_i <<"\n\n";
            lineMatches.push_back(make_pair(best_j,i));
        }
        /*
        vector<Line> l;
        l.push_back(lf);
        vector<Line> best;
        best.push_back(best_match);
        visualizeLineTrackCandidate(l,forwframe_->img,"forwframe_");
        visualizeLineTrackCandidate(best,curframe_->img,"curframe_best");
        visualizeLineTrackCandidate(candidate,curframe_->img,"curframe_");
        cv::waitKey(0);
        */
    }

}

//#define NLT
#ifdef  NLT
void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;
    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
    }

    // step 1: line extraction
    TicToc t_li;
    int lineMethod = 2;
    bool isROI = false;
    lineDetector ld(lineMethod, isROI, 0, (float)img.cols, 0, (float)img.rows);
    //ROS_INFO("ld inition costs: %fms", t_li.toc());
    TicToc t_ld;
    forwframe_->vecLine = ld.detect(img);

    for (size_t i = 0; i < forwframe_->vecLine.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id
    }
    ROS_INFO("line detect costs: %fms", t_ld.toc());

    // step 3: junction & line matching
    if(curframe_->vecLine.size() > 0)
    {
        TicToc t_nlt;
        vector<pair<int, int> > linetracker;
        NearbyLineTracking(forwframe_->vecLine, curframe_->vecLine, linetracker);
        ROS_INFO("line match costs: %fms", t_nlt.toc());

        // 对新图像上的line赋予id值
        for(int j = 0; j < linetracker.size(); j++)
        {
            forwframe_->lineID[linetracker[j].second] = curframe_->lineID[linetracker[j].first];
        }

        // show NLT match
        //visualizeLineMatch(curframe_->vecLine, forwframe_->vecLine, linetracker,
                           curframe_->img, forwframe_->img, "NLT Line Matches", 10, true,
                           "frame");
        //visualizeLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,"forwframe_");
        //visualizeLinewithID(curframe_->vecLine,curframe_->lineID,curframe_->img,"curframe_");
        stringstream ss;
        ss <<"/home/hyj/datasets/line/" <<frame_cnt<<".jpg";
        // SaveFrameLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,ss.str().c_str());
        waitKey(5);


        vector<Line> vecLine_tracked, vecLine_new;
        vector< int > lineID_tracked, lineID_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->vecLine[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
            }
            else
            {
                vecLine_tracked.push_back(forwframe_->vecLine[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
            }
        }
        int diff_n = 30 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
        if( diff_n > 0)    // 补充线条
        {
            for (int k = 0; k < vecLine_new.size(); ++k) {
                vecLine_tracked.push_back(vecLine_new[k]);
                lineID_tracked.push_back(lineID_new[k]);
            }
        }

        forwframe_->vecLine = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;

    }
    curframe_ = forwframe_;
}
#endif
int frame_num = 0;
#define MATCHES_DIST_THRESHOLD 30
void visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1, std::vector<KeyLine>octave0_2,
                          std::vector<DMatch> good_matches)
{
    //	Mat img_1;
    cv::Mat img1,img2;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;
    }

    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( good_matches.size(), 1 );
    drawLineMatches( img1, octave0_1, img2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ),Scalar::all( -1 ), lsd_mask,DrawLinesMatchesFlags::DEFAULT );
    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < good_matches.size(); ++k) {
        DMatch mt = good_matches[k];

        KeyLine line1 = octave0_1[mt.queryIdx];  // trainIdx
        KeyLine line2 = octave0_2[mt.trainIdx];  //queryIdx


        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
        cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
        cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);

    }
    /* plot matches */
    // cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);

    namedWindow("LSD matches", CV_WINDOW_NORMAL);
    imshow( "LSD matches", lsd_outImg );
    string name = to_string(frame_num);
    string path = "/home/dragon/ros_ws/p_ws/src/PL-VIO/feature_tracker/src/image/";
    name = path + name + ".jpg";
    frame_num ++;
    imwrite(name, lsd_outImg);
    // namedWindow("LSD matches1", CV_WINDOW_NORMAL);
    namedWindow("LSD matches2", CV_WINDOW_NORMAL);
    // imshow("LSD matches1", img1);
    imshow("LSD matches2", img2);
    waitKey(1);
}

void visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1, std::vector<KeyLine>octave0_2,
                          std::vector<bool> good_matches)
{
    //	Mat img_1;
    cv::Mat img1,img2;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;
    }

    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < good_matches.size(); ++k) {

        if(!good_matches[k]) continue;

        KeyLine line1 = octave0_1[k];  // trainIdx
        KeyLine line2 = octave0_2[k];  //queryIdx

        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
        cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
        cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);

    }
    /* plot matches */
    /*
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow( "LSD matches", lsd_outImg );
    */
   namedWindow("LSD matches1", CV_WINDOW_NORMAL);
   namedWindow("LSD matches2", CV_WINDOW_NORMAL);
    imshow("LSD matches1", img1);
    imshow("LSD matches2", img2);
    waitKey(1);
}
void visualize_line(Mat imageMat1,std::vector<KeyLine> octave0_1)
{
    //	Mat img_1;
    cv::Mat img1;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < octave0_1.size(); ++k) {

        unsigned int r = 255; //lowest + int(rand() % range);
        unsigned int g = 255; //lowest + int(rand() % range);
        unsigned int b = 0;  //lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(octave0_1[k].startPointX), int(octave0_1[k].startPointY));
        cv::Point endPoint = cv::Point(int(octave0_1[k].endPointX), int(octave0_1[k].endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);
        // cv::circle(img1, startPoint, 2, cv::Scalar(255, 0, 0), 5);
        // cv::circle(img1, endPoint, 2, cv::Scalar(0, 255, 0), 5);


    }
    /* plot matches */
    /*
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow( "LSD matches", lsd_outImg );
    */
    //namedWindow("LSD_C", CV_WINDOW_NORMAL);
    //imshow("LSD_C", img1);
    //waitKey(1);
}

cv::Mat last_unsuccess_image;
vector< KeyLine > last_unsuccess_keylsd;
vector< int >  last_unsuccess_id;
Mat last_unsuccess_lbd_descr;
void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;

    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);



//    cv::imshow("lineimg",img);
//    cv::waitKey(1);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
    }
    TicToc t_li;
    Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
    // lsd parameters
    line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine       = 1;     //1     	The way found lines will be refined
    opts.scale        = 0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
    opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
    opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
    opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
    opts.log_eps      = 1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
    opts.density_th   = 0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
    opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
    double min_line_length = 0.125;  // Line segments shorter than that are rejected
    // opts.refine       = 1;
    // opts.scale        = 0.5;
    // opts.sigma_scale  = 0.6;
    // opts.quant        = 2.0;
    // opts.ang_th       = 22.5;
    // opts.log_eps      = 1.0;
    // opts.density_th   = 0.6;
    // opts.n_bins       = 1024;
    // double min_line_length = 0.125;
    opts.min_length   = min_line_length*(std::min(img.cols,img.rows));

    std::vector<KeyLine> lsd, keylsd;
	//void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves, const std::vector<Mat>& masks ) const
    lsd_->detect( img, lsd, 2, 1, opts);
    // visualize_line(img,lsd);
    // step 1: line extraction
    // TicToc t_li;
    // std::vector<KeyLine> lsd, keylsd;
    // Ptr<LSDDetector> lsd_;
    // lsd_ = cv::line_descriptor::LSDDetector::createLSDDetector();
    // lsd_->detect( img, lsd, 2, 2 );

    sum_time += t_li.toc();
   ROS_INFO("line detect costs: %fms", t_li.toc());

    Mat lbd_descr, keylbd_descr;
    // step 2: lbd descriptor
    TicToc t_lbd;
    Ptr<BinaryDescriptor> bd_ = BinaryDescriptor::createBinaryDescriptor(  );
    

    bd_->compute( img, lsd, lbd_descr );
    // std::cout<<"lbd_descr = "<<lbd_descr.size()<<std::endl;
//////////////////////////
    for ( int i = 0; i < (int) lsd.size(); i++ )
    {
        if( lsd[i].octave == 0 && lsd[i].lineLength >= 60)
        {
            keylsd.push_back( lsd[i] );
            keylbd_descr.push_back( lbd_descr.row( i ) );
        }
    }
    // std::cout<<"lbd_descr = "<<lbd_descr.size()<<std::endl;
//    ROS_INFO("lbd_descr detect costs: %fms", keylsd.size() * t_lbd.toc() / lsd.size() );
    sum_time += keylsd.size() * t_lbd.toc() / lsd.size();
///////////////

    forwframe_->keylsd = keylsd;
    forwframe_->lbd_descr = keylbd_descr;

    for (size_t i = 0; i < forwframe_->keylsd.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id
    }
    
    // if(!first_img)
    // {
    //     std::vector<DMatch> lsd_matches;
    //     Ptr<BinaryDescriptorMatcher> bdm_;
    //     bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    //     bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
    //     visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, lsd_matches);
    //     // std::cout<<"lsd_matches = "<<lsd_matches.size()<<" forwframe_->keylsd = "<<keylbd_descr.size()<<" curframe_->keylsd = "<<keylbd_descr.size()<<std::endl;
    // }


    if(curframe_->keylsd.size() > 0)
    {
        /* compute matches */
        TicToc t_match;
        std::vector<DMatch> lsd_matches;
        Ptr<BinaryDescriptorMatcher> bdm_;
        bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
//        ROS_INFO("lbd_macht costs: %fms", t_match.toc());
        sum_time += t_match.toc();
        mean_time = sum_time/frame_cnt;
        // ROS_INFO("line feature tracker mean costs: %fms", mean_time);
        /* select best matches */
        std::vector<DMatch> good_matches;
        std::vector<KeyLine> good_Keylines;
        good_matches.clear();
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < 30 ){

                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd[mt.queryIdx] ;
                KeyLine line2 =  curframe_->keylsd[mt.trainIdx] ;
                Point2f serr = line1.getStartPoint() - line2.getEndPoint();
                Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                // std::cout<<"11111111111111111 = "<<abs(line1.angle-line2.angle)<<std::endl;
                if((serr.dot(serr) < 200 * 200) && (eerr.dot(eerr) < 200 * 200)&&abs(line1.angle-line2.angle)<0.1)   // 线段在图像里不会跑得特别远
                    good_matches.push_back( lsd_matches[i] );
            }
        }


        vector< int > success_id;
        // std::cout << forwframe_->lineID.size() <<" " <<curframe_->lineID.size();
        for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
            forwframe_->lineID[mt.queryIdx] = curframe_->lineID[mt.trainIdx];
            success_id.push_back(curframe_->lineID[mt.trainIdx]);
        }



        //visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, good_matches);

        //把没追踪到的线存起来

        vector<KeyLine> vecLine_tracked, vecLine_new;
        vector< int > lineID_tracked, lineID_new;
        Mat DEscr_tracked, Descr_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->keylsd[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
                Descr_new.push_back( forwframe_->lbd_descr.row( i ) );
            }
            
            else
            {
                vecLine_tracked.push_back(forwframe_->keylsd[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
                DEscr_tracked.push_back( forwframe_->lbd_descr.row( i ) );
            }
        }

        vector<KeyLine> h_Line_new, v_Line_new;
        vector< int > h_lineID_new,v_lineID_new;
        Mat h_Descr_new,v_Descr_new;
        for (size_t i = 0; i < vecLine_new.size(); ++i)
        {
            if((((vecLine_new[i].angle >= 3.14/4 && vecLine_new[i].angle <= 3*3.14/4))||(vecLine_new[i].angle <= -3.14/4 && vecLine_new[i].angle >= -3*3.14/4)))
            {
                h_Line_new.push_back(vecLine_new[i]);
                h_lineID_new.push_back(lineID_new[i]);
                h_Descr_new.push_back(Descr_new.row( i ));
            }
            else
            {
                v_Line_new.push_back(vecLine_new[i]);
                v_lineID_new.push_back(lineID_new[i]);
                v_Descr_new.push_back(Descr_new.row( i ));
            }      
        }
        int h_line,v_line;
        h_line = v_line =0;
        for (size_t i = 0; i < vecLine_tracked.size(); ++i)
        {
            if((((vecLine_tracked[i].angle >= 3.14/4 && vecLine_tracked[i].angle <= 3*3.14/4))||(vecLine_tracked[i].angle <= -3.14/4 && vecLine_tracked[i].angle >= -3*3.14/4)))
            {
                h_line ++;
            }
            else
            {
                v_line ++;
            }
        }
        int diff_h = 35 - h_line;
        int diff_v = 35 - v_line;

        // std::cout<<"h_line = "<<h_line<<" v_line = "<<v_line<<std::endl;
        if( diff_h > 0)    // 补充线条
        {
            int kkk = 1;
            if(diff_h > h_Line_new.size())
                diff_h = h_Line_new.size();
            else 
                kkk = int(h_Line_new.size()/diff_h);
            for (int k = 0; k < diff_h; ++k) 
            {
                vecLine_tracked.push_back(h_Line_new[k]);
                lineID_tracked.push_back(h_lineID_new[k]);
                DEscr_tracked.push_back(h_Descr_new.row(k));
            }
            // std::cout  <<"h_kkk = " <<kkk<<" diff_h = "<<diff_h<<" h_Line_new.size() = "<<h_Line_new.size()<<std::endl;
        }
        if( diff_v > 0)    // 补充线条
        {
            int kkk = 1;
            if(diff_v > v_Line_new.size())
                diff_v = v_Line_new.size();
            else 
                kkk = int(v_Line_new.size()/diff_v);
            for (int k = 0; k < diff_v; ++k)  
            {
                vecLine_tracked.push_back(v_Line_new[k]);
                lineID_tracked.push_back(v_lineID_new[k]);
                DEscr_tracked.push_back(v_Descr_new.row(k));
            }            // std::cout  <<"v_kkk = " <<kkk<<" diff_v = "<<diff_v<<" v_Line_new.size() = "<<v_Line_new.size()<<std::endl;
        }
        // int diff_n = 50 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
        // if( diff_n > 0)    // 补充线条
        // {
        //     for (int k = 0; k < vecLine_new.size(); ++k) {
        //         vecLine_tracked.push_back(vecLine_new[k]);
        //         lineID_tracked.push_back(lineID_new[k]);
        //         DEscr_tracked.push_back(Descr_new.row(k));
        //     }
        // }
        
        forwframe_->keylsd = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;
        forwframe_->lbd_descr = DEscr_tracked;

    }

    // 将opencv的KeyLine数据转为季哥的Line
    for (int j = 0; j < forwframe_->keylsd.size(); ++j) {
        Line l;
        KeyLine lsd = forwframe_->keylsd[j];
        l.StartPt = lsd.getStartPoint();
        l.EndPt = lsd.getEndPoint();
        l.length = lsd.lineLength;
        forwframe_->vecLine.push_back(l);
    }
    curframe_ = forwframe_;


}
