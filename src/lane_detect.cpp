#include "lane_detect.hpp"

#include <cassert>

#define PATH "/home/jetson/catkin_ws/logfiles/"

using namespace std;
using namespace cv;

namespace {
	// start must be less than end, so will never fail to find
	// a maximum element.
	int findMax(const std::vector<int>& values, int start, int end) {
		assert(start >= 0 && start < end && end <= values.size());
		int i_max = start;
		int max = values[start];
		for (int i = start + 1; i < end; i++)
			if (values[i] > max) {
				i_max = i;
				max = values[i];
			}
		return i_max;
	}
}

LaneDetector::LaneDetector(ros::NodeHandle nh)
  : nodeHandle_(nh) {  
    	/******* recording log *******/	  
	gettimeofday(&start_, NULL);

    	/******* Camera  calibration *******/
	double matrix[9], dist_coef[5];
	nodeHandle_.param("lane_detector/calibration/matrix/a",matrix[0], 3.2918100682757097e+02);
	nodeHandle_.param("lane_detector/calibration/matrix/b",matrix[1], 0.);
	nodeHandle_.param("lane_detector/calibration/matrix/c",matrix[2], 320.);
	nodeHandle_.param("lane_detector/calibration/matrix/d",matrix[3], 0.);
	nodeHandle_.param("lane_detector/calibration/matrix/e",matrix[4], 3.2918100682757097e+02);
	nodeHandle_.param("lane_detector/calibration/matrix/f",matrix[5], 240.);
	nodeHandle_.param("lane_detector/calibration/matrix/g",matrix[6], 0.);
	nodeHandle_.param("lane_detector/calibration/matrix/h",matrix[7], 0.);
	nodeHandle_.param("lane_detector/calibration/matrix/i",matrix[8], 1.);

	nodeHandle_.param("lane_detector/calibration/dist_coef/a",dist_coef[0], -3.2566540239089398e-01);
	nodeHandle_.param("lane_detector/calibration/dist_coef/b",dist_coef[1], 1.1504807178349362e-01);
	nodeHandle_.param("lane_detector/calibration/dist_coef/c",dist_coef[2], 0.);
	nodeHandle_.param("lane_detector/calibration/dist_coef/d",dist_coef[3], 0.);
	nodeHandle_.param("lane_detector/calibration/dist_coef/e",dist_coef[4], -2.1908791800876997e-02);

	Mat camera_matrix = Mat::eye(3, 3, CV_64FC1);
	Mat dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
	camera_matrix = (Mat1d(3, 3) << matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]);
	dist_coeffs = (Mat1d(1, 5) << dist_coef[0], dist_coef[1], dist_coef[2], dist_coef[3], dist_coef[4]);
	initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(), camera_matrix, Size(640, 480), CV_32FC1, map1_, map2_);

	/********** PID control ***********/
	prev_err_ = 0;

	last_Llane_base_ = 0;
	last_Rlane_base_ = 0;
	left_coef_ = Mat::zeros(3, 1, CV_32F);
	right_coef_ = Mat::zeros(3, 1, CV_32F);
	center_coef_ = Mat::zeros(3, 1, CV_32F);

	nodeHandle_.param("lane_detector/roi/width", width_, 640);
	nodeHandle_.param("lane_detector/roi/height", height_, 480);
	center_position_ = width_/2;

	e_values_.resize(3);

	corners_.resize(4);
	warpCorners_.resize(4);

	float t_gap, b_gap, t_height, b_height, f_extra, b_extra;
	int top_gap, bot_gap, top_height, bot_height, extra, extra_up, extra_down;
	nodeHandle_.param("lane_detector/roi/top_gap",t_gap, 0.336f);
	nodeHandle_.param("lane_detector/roi/bot_gap",b_gap, 0.078f);
	nodeHandle_.param("lane_detector/roi/top_height",t_height, 0.903f);
	nodeHandle_.param("lane_detector/roi/bot_height",b_height, 0.528f);
	nodeHandle_.param("lane_detector/roi/extra_f",f_extra, 0.0f);
	nodeHandle_.param("lane_detector/roi/extra_b",b_extra, 0.0f);
	nodeHandle_.param("lane_detector/roi/extra_up",extra_up, 0);
	nodeHandle_.param("lane_detector/roi/extra_down",extra_down, 0);
	nodeHandle_.param("lane_detector/roi/dynamic_roi",option_, true);
	nodeHandle_.param("lane_detector/roi/threshold",threshold_, 128);
	distance_ = 0;
	top_gap = width_ * t_gap; 
	bot_gap = width_ * b_gap;
	top_height = height_ * t_height;
	bot_height = height_ * b_height;

	/* ROI corner points  */
	corners_[0] = Point2f(top_gap+f_extra, bot_height);
	corners_[1] = Point2f((width_ - top_gap)+f_extra, bot_height);
	corners_[2] = Point2f(bot_gap+b_extra, top_height);
	corners_[3] = Point2f((width_ - bot_gap)+b_extra, top_height);
	
	wide_extra_upside_ = extra_up;
	wide_extra_downside_ = extra_down;
	
	/* Spread of ROI corner points */
	warpCorners_[0] = Point2f(wide_extra_upside_, 0.0);
	warpCorners_[1] = Point2f(width_ - wide_extra_upside_, 0.0);
	warpCorners_[2] = Point2f(wide_extra_downside_, height_);
	warpCorners_[3] = Point2f(width_ - wide_extra_downside_, height_);

	/* Lateral Control coefficient */
	nodeHandle_.param("lane_detector/K", K_, 0.15f);
	nodeHandle_.param("lane_detector/a/a", a_[0], 0.);
	nodeHandle_.param("lane_detector/a/b", a_[1], -0.37169);
	nodeHandle_.param("lane_detector/a/c", a_[2], 1.2602);
	nodeHandle_.param("lane_detector/a/d", a_[3], -1.5161);
	nodeHandle_.param("lane_detector/a/e", a_[4], 0.70696);
	nodeHandle_.param("lane_detector/b/a", b_[0], 0.);
	nodeHandle_.param("lane_detector/b/b", b_[1], -1.7536);
	nodeHandle_.param("lane_detector/b/c", b_[2], 5.0931);
	nodeHandle_.param("lane_detector/b/d", b_[3], -4.9047);
	nodeHandle_.param("lane_detector/b/e", b_[4], 1.6722);

	LoadParams();
}

LaneDetector::~LaneDetector(void) {
	clear_release();
}

void LaneDetector::LoadParams(void){
	nodeHandle_.param("lane_detector/eL_height",eL_height_, 1.0f);	
	nodeHandle_.param("lane_detector/e1_height",e1_height_, 1.0f);	
	nodeHandle_.param("lane_detector/trust_height",trust_height_, 1.0f);	
	nodeHandle_.param("lane_detector/lp",lp_, 756.0f);	
	nodeHandle_.param("lane_detector/steer_angle",SteerAngle_, 0.0f);
}

Mat LaneDetector::warped_img(Mat _frame) {
	Mat result, trans;
	trans = getPerspectiveTransform(corners_, warpCorners_);
	warpPerspective(_frame, result, trans, Size(width_, height_));

	return result;
}

Mat LaneDetector::warped_back_img(Mat _frame) {
	Mat result, trans;
	trans = getPerspectiveTransform(warpCorners_, corners_);
	warpPerspective(_frame, result, trans, Size(width_, height_));

	return result;
}

int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
	int max_index = -1;
	int max_val = 0;

	if (end > Max)
		end = Max;

	for (int i = start; i < end; i++) {
		if (max_val < hist[i]) {
			max_val = hist[i];
			max_index = i;
		}
	}
	if (max_index == -1) {
		cout << "ERROR : hist range" << endl;
		return -1;
	}
	return max_index;
}

Mat LaneDetector::polyfit(const vector<int>& xs, const vector<int>& ys) {
	Mat coef(3, 1, CV_32F);
	int N = (int) xs.size();
	int n = 2;
	// Vector that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	std::vector<double> X(2 * n + 1);
	for (int i = 0; i < (2 * n + 1); i++)
	{
		X[i] = 0;
		//consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		for (int j = 0; j < N; j++)
			X[i] = X[i] + pow(xs[j], i);        
	}

	// B is the Normal matrix(augmented) that will store the
	// equations, 
	std::vector<std::vector<double>> B(n + 1);
	for (int i = 0; i < (n + 1); i++)
	 	B[i] = std::vector<double>(n + 2);

	// Build the Normal matrix by storing the corresponding coefficients at
	// the right positions except the last column of the matrix
	for (int i = 0; i <= n; i++)
		for (int j = 0; j <= n; j++)
			B[i][j] = X[i + j];

	//Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	std::vector<double> Y(n + 1);
	for (int i = 0; i < (n + 1); i++)
	{
			
		Y[i] = 0;
		//consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		for (int j = 0; j < N; j++)
			Y[i] = Y[i] + pow(xs[j], i) * ys[j];
	}

	//load the values of Y as the last column of B(Normal Matrix but augmented)
	for (int i = 0; i <= n; i++)
		B[i][n + 1] = Y[i];                
	// n is made n+1 because the Gaussian Elimination part below
	// was for n equations, but here n is the degree of polynomial
	// and for n degree we get n+1 equations
	n = n + 1;

	//From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
	for (int i = 0; i < n; i++)                    
		for (int k = i + 1; k < n; k++)
			if (B[i][i] < B[k][i])
				for (int j = 0; j <= n; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	//loop to perform the gauss elimination
	for (int i = 0; i < (n - 1); i++)            
		for (int k = i + 1; k < n; k++)
		{
			double t = B[k][i] / B[i][i];
			//make the elements below the pivot elements equal to
			// zero or elimnate the variables
			for (int j = 0; j <= n; j++)
				B[k][j] = B[k][j] - t * B[i][j];
		}

	// 'a' is for value of the final coefficients
	std::vector<double> a(n + 1);
	for (int i = n - 1; i >= 0; i--) { //back-substitution
		a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
		for (int j = 0; j < n; j++)
			if (j != i)                //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
				a[i] = a[i] - B[i][j] * a[j];
		a[i] = a[i] / B[i][i];         //now finally divide the rhs by the coefficient of the variable to be calculated
		coef.at<float>(i, 0) = (float)a[i];
	}

	return coef;
}

Mat LaneDetector::detect_lines_sliding_window(Mat _frame, bool _view) {
	Mat frame, result;
	int width = _frame.cols;
	int height = _frame.rows;

	_frame.copyTo(frame);
	
	Mat nonZero;
	findNonZero(frame, nonZero);

	vector<int> good_left_inds;
	vector<int> good_right_inds;

	std::vector<int> hist(width, 0);
	for (int y = (height / 2); y < height; y++) // hist 범위 절반부터 읽기
		for (int x = 0; x < width; x++)
			if (frame.at<uchar>(y, x) == 255)
				hist[x] += 1;
	
	cvtColor(frame, result, COLOR_GRAY2BGR);

	int mid_point = width / 2; // 320
	int quarter_point = mid_point / 2; // 160
	int n_windows = 9;
	int margin = 120 * width / 1280;
	int min_pix = 30 * width / 1280;

	int window_width = margin * 2;	// 120
	int window_height;
	int distance;
	if (option_) {
		window_height = (height >= distance_) ? ((height-distance_) / n_windows) : (height / n_windows);	// defalut = 53
		distance = distance_;
	} else {
		distance = 0;
			window_height = height / n_windows;
	}
	int offset = margin;
	int range = 120 / 4;
	int Llane_base = findMax(hist, 100, mid_point);
	int Rlane_base = findMax(hist, mid_point, width - 100);

	int Llane_current = Llane_base;
	int Rlane_current = Rlane_base;

	int L_prev =  Llane_current;
	int R_prev =  Rlane_current;
	int L_gap = 0;
	int R_gap = 0;

	unsigned int index;

	for (int window = 0; window < n_windows; window++) {
		int	Ly_pos = height - (window + 1) * window_height - 1;
		int	Ry_pos = height - (window + 1) * window_height - 1;
		int	Ly_top = height - window * window_height;
		int	Ry_top = height - window * window_height;

		int	Lx_pos = Llane_current - margin;
		int	Rx_pos = Rlane_current - margin;
		if (_view) {
			rectangle(result, \
				Rect(Lx_pos, Ly_pos, window_width, window_height), \
				Scalar(255, 50, 100), 1);
			rectangle(result, \
				Rect(Rx_pos, Ry_pos, window_width, window_height), \
				Scalar(100, 50, 255), 1);
		}

		good_left_inds.clear();
		good_right_inds.clear();

		for (unsigned int index = (nonZero.total() - 1); index > 1; index--) {
			int nZ_y = nonZero.at<Point>(index).y;
			int nZ_x = nonZero.at<Point>(index).x;
			if ((nZ_y >= Ly_pos) && \
				(nZ_y > (distance)) && \
				(nZ_y < Ly_top) && \
				(nZ_x >= Lx_pos) && \
				(nZ_x < (Lx_pos + window_width))) {
				if (_view) {
					result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
					result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
					result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
				}
				good_left_inds.push_back(index);
			}
			
			if ((nZ_y >= (Ry_pos)) && \
				(nZ_y > (distance)) && \
				(nZ_y < Ry_top) && \
				(nZ_x >= Rx_pos) && \
				(nZ_x < (Rx_pos + window_width))) {
				if (_view) {
					result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
					result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
					result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
				}
				good_right_inds.push_back(index);
			}
		}
		
		int Lsum, Rsum;
		Lsum = Rsum = 0;
		unsigned int _size;
		vector<int> Llane_x;
		vector<int> Llane_y;
		vector<int> Rlane_x;
		vector<int> Rlane_y;

		if (good_left_inds.size() > (size_t)min_pix) {
			_size = (unsigned int)(good_left_inds.size());
			for (int i = Ly_top-1; i >= Ly_pos ; i--)
			{
				int Ly_sum = 0;
				int count = 0;
				for (index = 0; index < _size; index++) {
					int j = nonZero.at<Point>(good_left_inds.at(index)).y;
					if(i == j)
					{
						Ly_sum += nonZero.at<Point>(good_left_inds.at(index)).x;
						count++;
						Lsum += nonZero.at<Point>(good_left_inds.at(index)).x;
					}
				}
				if(count != 0)
				{
					left_x_.insert(left_x_.end(), Ly_sum/count);
					left_y_.insert(left_y_.end(), i);
					Llane_x.insert(Llane_x.end(), Ly_sum/count);
					Llane_y.insert(Llane_y.end(), i);
				} else {
					Llane_x.insert(Llane_x.end(), -1);
					Llane_y.insert(Llane_y.end(), i);
				}
			}
			Llane_current = Lsum / _size;
		} else{
			Llane_current += (L_gap);
		}
		if (good_right_inds.size() > (size_t)min_pix) {
			_size = (unsigned int)(good_right_inds.size());
			for (int i = Ry_top - 1 ; i >= Ry_pos ; i--)
			{
				int Ry_sum = 0;
				int count = 0;
				for (index = 0; index < _size; index++) {
					int j = nonZero.at<Point>(good_right_inds.at(index)).y;
					if(i == j)
					{
						Ry_sum += nonZero.at<Point>(good_right_inds.at(index)).x;
						count++;
						Rsum += nonZero.at<Point>(good_right_inds.at(index)).x;
					}
				}
				if(count != 0)
				{
					right_x_.insert(right_x_.end(), Ry_sum/count);
					right_y_.insert(right_y_.end(), i);
					Rlane_x.insert(Rlane_x.end(), Ry_sum/count);
					Rlane_y.insert(Rlane_y.end(), i);
				} else {
					Rlane_x.insert(Rlane_x.end(), -1);
					Rlane_y.insert(Rlane_y.end(), i);
				}
			}
			Rlane_current = Rsum / _size;
		} else{
			Rlane_current += (R_gap);
		}
		if (window != 0) {	
			if (Rlane_current != R_prev) {
				R_gap = (Rlane_current - R_prev);
			}
			if (Llane_current != L_prev) {
				L_gap = (Llane_current - L_prev);
			}
		}
		if ((Lsum != 0) && (Rsum != 0)) {
			for (int i = 0; i < Llane_x.size() ; i++)
			{
				if((Llane_x.at(i) != -1) && (Rlane_x.at(i) != -1)) {
					center_x_.insert(center_x_.end(), (Llane_x.at(i)+Rlane_x.at(i)) / 2 );
					center_y_.insert(center_y_.end(), Llane_y.at(i));
				}
			}
		}
		L_prev = Llane_current;
		R_prev = Rlane_current;
	}

	if (!left_x_.empty()) {
		left_coef_ = polyfit(left_y_, left_x_);
	}
	if (!right_x_.empty()) {
		right_coef_ = polyfit(right_y_, right_x_);
	}	
	if (!center_x_.empty()) {
		center_coef_ = polyfit(center_y_, center_x_);
	}

	return result;
}
Mat LaneDetector::draw_lane(Mat _sliding_frame, Mat _frame) {
	Mat new_frame, left_coef(left_coef_), right_coef(right_coef_), center_coef(center_coef_), trans;
	if (left_coef.empty() || right_coef.empty())
		return _frame;

	trans = getPerspectiveTransform(warpCorners_, corners_);
	_frame.copyTo(new_frame);

	vector<Point> left_point;
	vector<Point> right_point;
	vector<Point> center_point;

	vector<Point2f> left_point_f;
	vector<Point2f> right_point_f;
	vector<Point2f> center_point_f;

	vector<Point2f> warped_left_point;
	vector<Point2f> warped_right_point;
	vector<Point2f> warped_center_point;

	vector<Point> left_points;
	vector<Point> right_points;
	vector<Point> center_points;

	for (int i = 0; i <= height_; i++) {
		Point temp_left_point;
		Point temp_right_point;
		Point temp_center_point;

		temp_left_point.x = (int)((left_coef.at<float>(2, 0) * pow(i, 2)) + (left_coef.at<float>(1, 0) * i) + left_coef.at<float>(0, 0));
		temp_left_point.y = (int)i;
		temp_right_point.x = (int)((right_coef.at<float>(2, 0) * pow(i, 2)) + (right_coef.at<float>(1, 0) * i) + right_coef.at<float>(0, 0));
		temp_right_point.y = (int)i;
		temp_center_point.x = (int)((center_coef.at<float>(2, 0) * pow(i, 2)) + (center_coef.at<float>(1, 0) * i) + center_coef.at<float>(0, 0));
		temp_center_point.y = (int)i;

		left_point.push_back(temp_left_point);
		left_point_f.push_back(temp_left_point);
		right_point.push_back(temp_right_point);
		right_point_f.push_back(temp_right_point);
		center_point.push_back(temp_center_point);
		center_point_f.push_back(temp_center_point);
	}
	const Point* left_points_point_ = (const cv::Point*) Mat(left_point).data;
	int left_points_number_ = Mat(left_point).rows;
	const Point* right_points_point_ = (const cv::Point*) Mat(right_point).data;
	int right_points_number_ = Mat(right_point).rows;
	const Point* center_points_point_ = (const cv::Point*) Mat(center_point).data;
	int center_points_number_ = Mat(center_point).rows;

	polylines(_sliding_frame, &left_points_point_, &left_points_number_, 1, false, Scalar(255, 200, 200), 5);
	polylines(_sliding_frame, &right_points_point_, &right_points_number_, 1, false, Scalar(200, 200, 255), 5);
	polylines(_sliding_frame, &center_points_point_, &center_points_number_, 1, false, Scalar(200, 255, 200), 5);
	
	perspectiveTransform(left_point_f, warped_left_point, trans);
	perspectiveTransform(right_point_f, warped_right_point, trans);
	perspectiveTransform(center_point_f, warped_center_point, trans);

	for (int i = 0; i <= height_; i++) {
		Point temp_left_point;
		Point temp_right_point;
		Point temp_center_point;

		temp_left_point.x = (int)warped_left_point[i].x;
		temp_left_point.y = (int)warped_left_point[i].y;
		temp_right_point.x = (int)warped_right_point[i].x;
		temp_right_point.y = (int)warped_right_point[i].y;
		temp_center_point.x = (int)warped_center_point[i].x;
		temp_center_point.y = (int)warped_center_point[i].y;

		left_points.push_back(temp_left_point);
		right_points.push_back(temp_right_point);
		center_points.push_back(temp_center_point);
	}

	const Point* left_points_point = (const cv::Point*) Mat(left_points).data;
	int left_points_number = Mat(left_points).rows;
	const Point* right_points_point = (const cv::Point*) Mat(right_points).data;
	int right_points_number = Mat(right_points).rows;
	const Point* center_points_point = (const cv::Point*) Mat(center_points).data;
	int center_points_number = Mat(center_points).rows;
	
	polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(255, 100, 100), 5);
	polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(100, 100, 255), 5);
	polylines(new_frame, &center_points_point, &center_points_number, 1, false, Scalar(100, 255, 100), 5);
	
	left_point.clear();
	right_point.clear();
	center_point.clear();

	/***************/
	/* Dynamic ROI */
	/***************/

	Point temp_roi_point;
	Point temp_droi_point;
	vector<Point2f> droi_point_f;
	vector<Point2f> warped_droi_point;
	vector<Point> roi_points;
	vector<Point> droi_points;
	
	temp_droi_point.y = (int)height_;
	temp_droi_point.x = 0;
	droi_point_f.push_back(temp_droi_point);
	temp_droi_point.x = (int)width_;
	droi_point_f.push_back(temp_droi_point);
	
	temp_droi_point.y = distance_;
	temp_droi_point.x = (int)width_;
	droi_point_f.push_back(temp_droi_point);
	temp_droi_point.x = 0;
	droi_point_f.push_back(temp_droi_point);
	
	perspectiveTransform(droi_point_f, warped_droi_point, trans);
	
	int droi_num[5] = {0, 1, 2, 3, 0};
	int roi_num[5] = {0, 1, 3, 2, 0};
	
	for (int i = 0; i < 5; i++) {
		temp_droi_point.x = (int)warped_droi_point[droi_num[i]].x;
		temp_droi_point.y = (int)warped_droi_point[droi_num[i]].y;
		
		droi_points.push_back(temp_droi_point);
		
		temp_roi_point.x = (int)corners_[roi_num[i]].x;
		temp_roi_point.y = (int)corners_[roi_num[i]].y;
		
		roi_points.push_back(temp_roi_point);
	}

	const Point* roi_points_point = (const cv::Point*) Mat(roi_points).data;
	int roi_points_number = Mat(roi_points).rows;
	const Point* droi_points_point = (const cv::Point*) Mat(droi_points).data;
	int droi_points_number = Mat(droi_points).rows;

	polylines(_frame, &roi_points_point, &roi_points_number, 1, false, Scalar(0, 255, 0), 5);
	polylines(_frame, &droi_points_point, &droi_points_number, 1, false, Scalar(0, 0, 255), 5);

	static const string TEXT = "ROI";
	Point2f T_pos(Point2f(270, _frame.rows-120));
	putText(_frame, TEXT, T_pos, FONT_HERSHEY_DUPLEX, 2, Scalar(0, 0, 255), 5, 8);
	
	return new_frame;
}

void LaneDetector::clear_release() {
	left_lane_inds_.clear();
	right_lane_inds_.clear();
	left_x_.clear();
	left_y_.clear();
	right_x_.clear();
	right_y_.clear();
	center_x_.clear();
	center_y_.clear();
}

void LaneDetector::steer_error_log(){
	double time;
	float data;
	char fname[] = "SteerError00.csv";
	static char file[128] = {0x00, };
	static bool flag = false;
	ofstream writeFile;
	ifstream readFile;

	if(!flag) {
		for(int i = 0; i<100; i++)
		{
			fname[10] = i/10 + '0';
			fname[11] = i%10 + '0';
			sprintf(file, "%s%s", PATH, fname);
			readFile.open(file);
			if(readFile.fail()) {
				readFile.close();
				writeFile.open(file);
				break;
			}
			readFile.close();
		}
		writeFile << "time[s],e1[m]" << endl;
		flag = true;
	}
	else {
		writeFile.open(file, fstream::out | fstream::app);
		gettimeofday(&end_, NULL);
		time = (end_.tv_sec - start_.tv_sec) + ((end_.tv_usec - start_.tv_usec)/1000000.0);
		data = (e_values_[2]/831.17f);
		writeFile << time << ", " << data << endl;
	}

	writeFile.close();
}

void LaneDetector::get_steer_coef(float vel){
	float value = std::min(vel, 1.2f);

	if (value < 0.65f){
		K1_ = K2_ =  K_;
	}
	else{
		K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4];
		K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4];
	}
	
}

void LaneDetector::calc_curv_rad_and_center_dist() {
	Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_);
	if (l_fit.empty() || r_fit.empty())
		return;

	float car_position = width_ / 2;

	lane_coef_.right.c2 = r_fit.at<float>(2, 0);
	lane_coef_.right.c1 = r_fit.at<float>(1, 0);
	lane_coef_.right.c0 = r_fit.at<float>(0, 0);

	lane_coef_.left.c2 = l_fit.at<float>(2, 0);
	lane_coef_.left.c1 = l_fit.at<float>(1, 0);
	lane_coef_.left.c0 = l_fit.at<float>(0, 0);

	lane_coef_.center.c2 = c_fit.at<float>(2, 0);
	lane_coef_.center.c1 = c_fit.at<float>(1, 0);
	lane_coef_.center.c0 = c_fit.at<float>(0, 0);

	float i = ((float)height_) * eL_height_;	
	float j = ((float)height_) * trust_height_;
	float k = ((float)height_) * e1_height_;

	float l1 =  j - i;
	float l2 = ((lane_coef_.center.c2 * pow(i, 2)) + (lane_coef_.center.c1 * i) + lane_coef_.center.c0) -
		((lane_coef_.center.c2 * pow(j, 2)) + (lane_coef_.center.c1 * j) + lane_coef_.center.c0);

	e_values_[0] = ((lane_coef_.center.c2 * pow(i, 2)) + (lane_coef_.center.c1 * i) + lane_coef_.center.c0) - car_position;	//eL
	e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));	//trust_e1
	e_values_[2] = ((lane_coef_.center.c2 * pow(k, 2)) + (lane_coef_.center.c1 * k) + lane_coef_.center.c0) - car_position;	//e1
	SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
}

float LaneDetector::display_img(Mat _frame, int _delay, bool _view) {		
	Mat new_frame, gray_frame, edge_frame, binary_frame, sliding_frame, resized_frame;

	if(!_frame.empty()) resize(_frame, new_frame, Size(width_, height_));
	Mat trans = getPerspectiveTransform(corners_, warpCorners_);

	cuda::GpuMat gpu_map1, gpu_map2;
	gpu_map1.upload(map1_);
	gpu_map2.upload(map2_);

	cuda::GpuMat gpu_frame, gpu_remap_frame, gpu_warped_frame, gpu_blur_frame, gpu_gray_frame, gpu_binary_frame;
	cuda::GpuMat gpu_inverted_frame;
	gpu_frame.upload(new_frame);
	cuda::remap(gpu_frame, gpu_remap_frame, gpu_map1, gpu_map2, INTER_LINEAR);
	gpu_remap_frame.download(new_frame);
	cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, trans, Size(width_, height_));
	static cv::Ptr< cv::cuda::Filter > filters;
	filters = cv::cuda::createGaussianFilter(gpu_warped_frame.type(), gpu_blur_frame.type(), cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
	filters->apply(gpu_warped_frame, gpu_blur_frame);
	cuda::cvtColor(gpu_blur_frame, gpu_gray_frame, COLOR_BGR2GRAY);
	cuda::threshold(gpu_gray_frame, gpu_binary_frame, threshold_, 255, THRESH_BINARY);
	
	cuda::bitwise_not(gpu_binary_frame, gpu_inverted_frame);
	gpu_inverted_frame.download(gray_frame);
	sliding_frame = detect_lines_sliding_window(gray_frame, _view);
	calc_curv_rad_and_center_dist();

	if (_view) {
		resized_frame = draw_lane(sliding_frame, new_frame);
		
		namedWindow("Window1");
		moveWindow("Window1", 0, 0);
		namedWindow("Window2");
		moveWindow("Window2", 640, 0);
		namedWindow("Window3");
		moveWindow("Window3", 1280, 0);
			
		if(!new_frame.empty()) {
			resize(new_frame, new_frame, Size(640, 480));
			imshow("Window1", new_frame);
		}
		if(!sliding_frame.empty()) {
			resize(sliding_frame, sliding_frame, Size(640, 480));
			imshow("Window2", sliding_frame);
		}
		if(!resized_frame.empty()){
			resize(resized_frame, resized_frame, Size(640, 480));
			imshow("Window3", resized_frame);
		}

		waitKey(_delay);
	}
	clear_release();

	return SteerAngle_;
}
