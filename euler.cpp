
struct bbox{
    int x1, y1;   // x1, y1 左上角坐标,
    int x2, y2;   // x2, y2 右下角坐标
    float score;  // 置信度
    float landmark[28]; // 人脸关键点，xy，xy...
    float euler[3]; //头部姿态角，pitch，yaw，roll
    float rotation_vector[3]; //旋转矩阵
    float translation_vector[3]; //平移矩阵
};

void calc_pose(int rows, int cols,  std::vector < cv::Point2d > landmark_points, std::vector < cv::Point3d > object_pts, cv::Mat & rotation_vector, 
	cv::Mat & translation_vector, cv::Mat & euler_angle)
{
	// Camera internals
	double focal_length = cols;				// Approximate focal length.
	Point2d center		= cv::Point2d(cols / 2, rows / 2);
	cv::Mat cam_matrix	= (cv::Mat_ < double >
		 (3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, cv::DataType < double >::type); // Assuming no lens distortion


	//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	


	/*
		cv::Mat tmp(800, 800, CV_64FC3);
		for (unsigned int i = 0; i < 5; ++i)
		{
			cv::circle(tmp, cv::Point((object_pts[i].x+150), (object_pts[i].y+150)), 1, cv::Scalar(0, 255, 0), -1);
		}
		cv::imshow("xx", tmp);
		//cv::waitKey(0);

	*/
	//reprojected 2D points
	std::vector < cv::Point2d > reprojectdst;
	reprojectdst.resize(8);

	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);



/*
	//draw features
	for (unsigned int i = 0; i < landmark_points.size(); ++i)
		{
		cv::circle(temp, cv::Point(landmark_points[i].x, landmark_points[i].y), 4, cv::Scalar(0, 255, 0), -1);
		}*/

	//result
	//cv::Mat rotation_vector;						  //3 x 1
	cv::Mat rotation_mat;							//3 x 3 R

	//cv::Mat translation_vec;					   //3 x 1 T
	cv::Mat pose_mat	= cv::Mat(3, 4, CV_64FC1);	//3 x 4 R | T

	//cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);
	//calc pose
	cv::solvePnP(object_pts, landmark_points, cam_matrix, dist_coeffs, rotation_vector, translation_vector);

	//calc euler angle
	cv::Rodrigues(rotation_vector, rotation_mat);
	cv::hconcat(rotation_mat, translation_vector, pose_mat);
	cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);




}


void head_pose_estimate(int height, int width, bbox& box){
/*
					printf("w: %d  h: %d \n", box.x2 - box.x1 + 1, box.y2 - box.y1 + 1);
					
					cv::rectangle(im, cv::Rect(box.x1, box.y1, box.x2 - box.x1 + 1, box.y2 - box.y1 + 1), 
						cv::Scalar(0, 255, 0), 5, 1, 0);
		*/
					std::vector < cv::Point2d > image_pts;
		
#if 0
					for (int j = 0; j < LANDMARK_NUM; j += 2)
						{
						cv::circle(im, cv::Point(box.landmark[j], box.landmark[j + 1]), 3, cv::Scalar(0, 0, 255), -1);
						cv::putText(im, to_string(j/2), cv::Point(box.landmark[j], box.landmark[j + 1]), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255));
						
						printf("landmark %d,%d\n", box.landmark[j], box.landmark[j + 1]);
						image_pts.push_back(cv::Point2d(box.landmark[j], box.landmark[j + 1]));
						}
#endif

					// 14points of fdnet, pick from 98 points
					image_pts.push_back(cv::Point2d(box.landmark[ 4*2], box.landmark[ 4*2+1])); //#17 left brow left corner
					image_pts.push_back(cv::Point2d(box.landmark[ 5*2], box.landmark[ 5*2+1])); //#21 left brow right corner
					image_pts.push_back(cv::Point2d(box.landmark[ 7*2], box.landmark[ 7*2+1])); //#22 right brow left corner
					image_pts.push_back(cv::Point2d(box.landmark[ 6*2], box.landmark[ 6*2+1])); //#26 right brow right corner
					image_pts.push_back(cv::Point2d(box.landmark[ 0*2], box.landmark[ 0*2+1])); //#36 left eye left corner
					image_pts.push_back(cv::Point2d(box.landmark[ 1*2], box.landmark[ 1*2+1])); //#39 left eye right corner
					image_pts.push_back(cv::Point2d(box.landmark[ 2*2], box.landmark[ 2*2+1])); //#42 right eye left corner
					image_pts.push_back(cv::Point2d(box.landmark[ 3*2], box.landmark[ 3*2+1])); //#45 right eye right corner
					image_pts.push_back(cv::Point2d(box.landmark[12*2], box.landmark[12*2+1])); //#31 nose left corner
					image_pts.push_back(cv::Point2d(box.landmark[13*2], box.landmark[13*2+1])); //#35 nose right corner
					image_pts.push_back(cv::Point2d(box.landmark[ 8*2], box.landmark[ 8*2+1])); //#48 mouth left corner
					image_pts.push_back(cv::Point2d(box.landmark[ 9*2], box.landmark[ 9*2+1])); //#54 mouth right corner
					image_pts.push_back(cv::Point2d(box.landmark[10*2], box.landmark[10*2+1])); //#57 mouth central bottom corner
					image_pts.push_back(cv::Point2d(box.landmark[11*2], box.landmark[11*2+1])); //#8 chin corner

			
		
					// head pose estimate	
					cv::Mat rotation_vector;				// Rotation in axis-angle form
					cv::Mat translation_vector;
					cv::Mat euler_angle;
		
					//head_post_estimate(im, landmark_points, rotation_vector, translation_vector);
					//head_post_estimate2(im, landmark_points, rotation_vector, translation_vector, euler_angle);

					std::vector < cv::Point3d > object_pts;


	//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));	 //#33 left brow left corner
	object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));	 //#29 left brow right corner
	object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));	 //#34 right brow left corner
	object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));	 //#38 right brow right corner
	object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));	 //#13 left eye left corner
	object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));	 //#17 left eye right corner
	object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));	 //#25 right eye left corner
	object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));	 //#21 right eye right corner
	object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));	 //#55 nose left corner
	object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));	 //#49 nose right corner
	object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));	 //#43 mouth left corner
	object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));	 //#39 mouth right corner
	object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));	 //#45 mouth central bottom corner
	object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));	 //#6 chin corner


					calc_pose(height, width, image_pts, object_pts, rotation_vector, translation_vector, euler_angle);
#if 1

					box.euler[0] = euler_angle.at<double>(0);
					box.euler[1] = euler_angle.at<double>(1);
					box.euler[2] = euler_angle.at<double>(2);
					
					box.rotation_vector[0] = rotation_vector.at<double>(0);
					box.rotation_vector[1] = rotation_vector.at<double>(1);
					box.rotation_vector[2] = rotation_vector.at<double>(2);

					box.translation_vector[0] = translation_vector.at<double>(0);
					box.translation_vector[1] = translation_vector.at<double>(1);
					box.translation_vector[2] = translation_vector.at<double>(2);

					//draw_pose2(im, rotation_vector, translation_vector, image_pts, euler_angle);
#else	

	float euler_x = euler_angle.at<double>(0);
	float euler_y = euler_angle.at<double>(1);
	float euler_z = euler_angle.at<double>(2);
	
	box.euler[0] = abs(euler_x) <= 90 ? euler_x : -(180 - abs(euler_x))*euler_x / abs(euler_x);
	box.euler[1] = abs(euler_y) <= 90 ? euler_y : -(180 - abs(euler_y))*euler_y / abs(euler_y);
	box.euler[2] = abs(euler_z) <= 90 ? euler_z : -(180 - abs(euler_z))*euler_z / abs(euler_z);

#endif
					 

}


main(){
    
    // detect landmark, store in box
    
    //estimate
    head_pose_estimate(im.rows, im.cols, box);

}