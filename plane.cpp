#include "plane.h"

bool readFile(string filename)
{
	FileStorage fs(filename, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["width"] >> imageSize.width;
		fs["height"] >> imageSize.height;
		fs["cameraMatrix1"] >> cameraMatrix1;
		fs["distCoeffs1"] >> distCoeffs1;
		fs["cameraMatrix2"] >> cameraMatrix2;
		fs["distCoeffs2"] >> distCoeffs2;
		fs["R"] >> R;
		fs["T"] >> T;
		fs.release();


		/*cout << "Succeed to read the Calibration result!!!" << endl;
		cout << "ͼƬ��С   " << imageSize.width << "  " << imageSize.height << endl;
		cout << "������ڲξ���" << endl << cameraMatrix1 << endl;
		cout << "�������ξ���" << endl << distCoeffs1 << endl;
		cout << "������ڲξ���" << endl << cameraMatrix2 << endl;
		cout << "�������ξ���" << endl << distCoeffs2 << endl;
		cout << "R:" << endl << R << endl;
		cout << "T:" << endl << T << endl;*/

		return true;
	}
	else
	{
		cerr << "Error: can not open the Calibration result file!!!!!" << endl;
		return false;
	}
}

void stereo_rectify(Mat grayimgLeft, Mat grayimgRight)
{
	/**************************************************************************************/
	/*
	Ҫʵ������У����ʹ����ͼ��ƽ�����ж�׼����Ҫ�õ����²�����
	cameraMatrix1, distCoeffs1, R1, P1
	cameraMatrix2, distCoeffs2, R2, P2
	 Bouguet����
	���ַ�����Ϊ���궨����У�������������Ǹ�������궨��õ��ڲξ��󡢻���ϵ����R��T��Ϊ
	���룬����stereoRectify()�����õ�R1��P1��R2��P2��ֵ��
	*/
	/**************************************************************************************/
		//����alpha�����öԽ��Ӱ��ܴ�
		//alpha��ͼ�����ϵ����ȡֵ��Χ��-1��0~1��
		//��ȡֵΪ 0 ʱ��OpenCV���У�����ͼ��������ź�ƽ�ƣ�ʹ��remapͼ��ֻ��ʾ��Ч���أ���ȥ��������ı߽�����,�����ڻ����˱��ϵ�����Ӧ�ã�
		//��alphaȡֵΪ1ʱ��remapͼ����ʾ����ԭͼ���а��������أ���ȡֵ�����ڻ���ϵ�����ٵĸ߶�����ͷ��
		//alphaȡֵ��0-1֮��ʱ��OpenCV����Ӧ��������ԭͼ��ı߽��������ء�
		//AlphaȡֵΪ-1ʱ��OpenCV�Զ��������ź�ƽ��
	stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q,
		0, -1, imageSize, &validROI1, &validROI2);

	/*����stereoRectify���������R��P������ͼ���ӳ���mapx,mapy
	mapx,mapy������ӳ�����������Ը�remap()�������ã���У��ͼ��ʹ������ͼ���沢���ж�׼
	initUndistortRectifyMap()�Ĳ���newCameraMatrix����У��������������
	��openCV���棬У����ļ��������Mrect�Ǹ�ͶӰ����Pһ�𷵻صġ�
	�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
	*/
	initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, mapx1, mapy1);
	initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, mapx2, mapy2);

	Mat medianBlurLeft, medianBlurRight;
	medianBlur(grayimgLeft, medianBlurLeft, 5);
	medianBlur(grayimgRight, medianBlurRight, 5);

	Mat gaussianBlurLeft, gaussianBlurRight;
	GaussianBlur(medianBlurLeft, gaussianBlurLeft, Size(5, 5), 0, 0);
	GaussianBlur(medianBlurRight, gaussianBlurRight, Size(5, 5), 0, 0);

	//����remap֮�����������ͼ���Ѿ����沢���ж�׼�� 
	remap(gaussianBlurLeft, imgLeft, mapx1, mapy1, INTER_LINEAR);
	remap(gaussianBlurRight, imgRight, mapx2, mapy2, INTER_LINEAR);
}

void show_rectify_performance()
{
	/**********************************************************************************/
	/***************������ͼ���У�������ʾ��ͬһ�����Ͻ��жԱ�*********************/
	Mat canvas;
	double sf = 0.7;
	int w, h;
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC1);
	//��ͼ�񻭵�������
	//�õ���������벿�� 
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
	//��ͼ�����ŵ���canvasPartһ����С��ӳ�䵽����canvas��ROI������  
	resize(imgLeft, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	//��ͼ�񻭵������� 
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(imgRight, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	//���϶�Ӧ������
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	imshow("rectified", canvas);
	cv::waitKey(0);
	/**********************************************************************************/
}

vector<RotatedRect> findellipse(Mat LLL, string savePath)
{
	Mat src_img, dst_img;
	vector<RotatedRect> box2;
	src_img = LLL;
	//��ֵ�˲�
	blur(src_img, src_img, Size(3, 3));
	Canny(src_img, dst_img, cannythreshold, cannythreshold * 3);

	imwrite(savePath, dst_img.clone());

	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	findContours(dst_img, contours, hireachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1);
	for (unsigned int i = 0; i < contours.size(); i++)
	{

		double S1 = contourArea(contours[i]);
		if (S1 > 20)
		{
			Mat pointsf;
			Mat(contours[i]).convertTo(pointsf, CV_32F);
			RotatedRect box = fitEllipse(pointsf);
			if (box.size.width / box.size.height > 10 && box.size.width / box.size.height < 0.1)
				continue;
			double S2 = box.size.width * box.size.height * PI / 4;
			if (S1 / S2 > 0.95 && S1 / S2 < 1.1)
			{
				box2.push_back(box);
				circle(LLL, box.center, 2, Scalar(255));
				ellipse(LLL, box, Scalar(255), 1, 8);
			}
		}
	}
	return  box2;
}

void stereo_match(vector<RotatedRect> &Match_boxL, vector<RotatedRect> &Match_boxR, vector<RotatedRect> boxL, vector<RotatedRect> boxR, int y, int x, bool isShow)
{
	int K = 0;
	for (int i = 0; i < boxL.size(); i++) {
		//cout << "��ǰ��ͼ������" << boxL[i].center << endl;
		for (int j = 0; j < boxR.size(); j++) {
			if (abs(boxL[i].center.y - boxR[j].center.y) < y && abs(boxL[i].center.x - boxR[j].center.x) < imgLeft.cols / x) {
				//cout << "Y�ӽ�����ͼ��" << boxR[j].center << endl;
				K++;
				Match_boxL.push_back(boxL[i]);
				Match_boxR.push_back(boxR[j]);
				break;
			}
		}
		//cout << "---------" << endl;
	}
	cout << "ƥ�䵽�ĵ���" << K << endl;
	if (isShow) {
		Mat left_img = imread(savePath);
		Mat right_img = imread(savePath2);
		for (int i = 0; i < Match_boxL.size(); i++) {
			putText(left_img, to_string(i), Match_boxL[i].center, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
			putText(right_img, to_string(i), Match_boxR[i].center, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		}
		imshow("left", left_img);
		imshow("right", right_img);
		cv::waitKey(0);
	}
}

vector<Point3d> getWord3d(vector<RotatedRect> Match_boxL, vector<RotatedRect> Match_boxR) {
	//�����Ӳ�ͼ������ң����Ϊ���Ļ���ֵΪ-1���Һ������������õ�
	vector<double> disparity(Match_boxL.size());
	for (int i = 0; i < Match_boxL.size(); i++) {
		if ((Match_boxL[i].center.x - Match_boxR[i].center.x) > 0) {
			disparity[i] = Match_boxL[i].center.x - Match_boxR[i].center.x;
		}
		else {
			disparity[i] = -1;
		}
	}
	//pix4d[u,v,d,1]  
	vector<Mat> pix4d(Match_boxL.size());
	//word4d[Xw,Yw,Zw,W]
	vector<Mat> word4d(Match_boxL.size());
	//��������;
	vector<Point3d> word3d2(Match_boxL.size());
	int useNum = 0;
	for (int i = 0; i < Match_boxL.size(); i++) {
		pix4d[i].create(4, 1, CV_64FC1);
		word4d[i].create(4, 1, CV_64FC1);
		if (disparity[i] != -1) {
			pix4d[i].at<double>(0, 0) = Match_boxL[i].center.x;
			pix4d[i].at<double>(1, 0) = Match_boxL[i].center.y;
			pix4d[i].at<double>(2, 0) = disparity[i];
			pix4d[i].at<double>(3, 0) = 1;
			word4d[i] = Q * pix4d[i];
			word3d2[i].x = word4d[i].at<double>(0, 0) / word4d[i].at<double>(3, 0);
			word3d2[i].y = word4d[i].at<double>(1, 0) / word4d[i].at<double>(3, 0);
			word3d2[i].z = word4d[i].at<double>(2, 0) / word4d[i].at<double>(3, 0);
			useNum++;
		}
		else {
			word3d2[i].x = 0;
			word3d2[i].y = 0;
			word3d2[i].z = 0;
		}
	}
	cout << "ʵ�ʿ��õ���" << useNum << endl;
	vector<Point3d> word3d(useNum);
	for (int i = 0, j = 0; j < Match_boxL.size(); j++) {
		if (disparity[j] != -1) {
			word3d[i] = word3d2[j];
			i++;
		}
	}
	return word3d;
}

vec4d ransac(vector<Point3d> &pts_3d, int max_iter, double threshold)
{
	vec4d plane;
	srand(time(0)); //�������
	int size_old = 3;
	double a_final, b_final, c_final, d_final; //ƽ�淨����ϵ��
	while (--max_iter) //����ѭ���Ĵ���
	{
		vector<int> index;
		for (int k = 0; k < 3; ++k)
		{
			index.push_back(rand() % pts_3d.size()); //���ѡȡ������ 
		}
		auto idx = index.begin();
		double x1 = pts_3d.at(*idx).x, y1 = pts_3d.at(*idx).y, z1 = pts_3d.at(*idx).z;
		++idx;
		double x2 = pts_3d.at(*idx).x, y2 = pts_3d.at(*idx).y, z2 = pts_3d.at(*idx).z;
		++idx;
		double x3 = pts_3d.at(*idx).x, y3 = pts_3d.at(*idx).y, z3 = pts_3d.at(*idx).z;

		double a = (y3 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		double b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		double c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		double d = -(a * x1 + b * y1 + c * z1);

		for (auto iter = pts_3d.begin(); iter != pts_3d.end(); ++iter)
		{
			double dis = fabs(a*iter->x + b * iter->y + c * iter->z + d) / sqrt(a*a + b * b + c * c);//�㵽ƽ��ľ��빫ʽ
			if (dis < threshold) {
				index.push_back(iter - pts_3d.begin());
			}
		}
		//���¼���
		if (index.size() > size_old)
		{
			size_old = index.size();
			a_final = a; b_final = b; c_final = c; d_final = d;
		}
		//index.clear();
	}
	//cout << a_final << " " << b_final << " " << c_final << " " << d_final << endl;
	plane.A = a_final;
	plane.B = b_final;
	plane.C = c_final;
	plane.D = d_final;
	return plane;
}

vec4d plane_reconstruction() {
	//--------------------------------------��ȡ�������
	readFile(filename);


	//--------------------------------------����У��
	Mat grayimgLeft = imread(path1, IMREAD_GRAYSCALE);
	Mat grayimgRight = imread(path2, IMREAD_GRAYSCALE);


	//�õ�У�����imgLeft,imgRight
	stereo_rectify(grayimgLeft, grayimgRight);


	//չʾ
	if (is_show_rectify_performance)
		show_rectify_performance();



	//--------------------------------------�ҵ���ǵ�����
	vector<RotatedRect> boxL, boxR;
	boxL = findellipse(imgLeft, savePath);
	boxR = findellipse(imgRight, savePath2);
	cout << "�ҵ�����ͼ���ǵ�:" << boxL.size() << endl;
	cout << "�ҵ�����ͼ���ǵ�:" << boxR.size() << endl;


	//--------------------------------------��ǵ�ƥ��
	vector<RotatedRect> Match_boxL, Match_boxR;

	//ƥ���ǵ�
	stereo_match(Match_boxL, Match_boxR, boxL, boxR, y, x, is_show_stereo_match);


	//--------------------------------------��ǵ���ά�ؽ�
	vector<Point3d> word3d;
	word3d = getWord3d(Match_boxL, Match_boxR);
	cout << word3d << endl;


	//--------------------------------------��������ؽ�
	cout << "-------------------" << endl;
	vec4d plane;
	plane = ransac(word3d, max_iter, ransac_threshold);

	return plane;
}