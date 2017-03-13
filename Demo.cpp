#include <iostream>
#include <fstream>
#include <cv.hpp>
#include <string>
//#include <highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "package_bgs/PBAS/PixelBasedAdaptiveSegmenter.h"
#include "package_tracking/BlobTracking.h"
#include "package_analysis/GlochidiaCouting.h"
#include <windows.h>

cv::Mat frame;
int main(int argc, char **argv)
{ 
	MessageBox(NULL, "Counting Started", "Note", MB_OK);
	// std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;

	/* Open video file */
	cv::VideoCapture capture(argv[1]);
	if(!capture.isOpened()){
		std::cerr << "Cannot open video!" << std::endl;
		return 1;
	}
	/* Open file to write Glochidia Count */
	std::fstream file;
	file.open("total_count.txt", std::fstream::in | std::fstream::out);
	std::cout<<"success"<<std::endl;
	
	/* Background Subtraction Algorithm */
	IBGS *bgs;
	bgs = new PixelBasedAdaptiveSegmenter;

	/* Blob Tracking Algorithm */
	cv::Mat img_blob;
	BlobTracking* blobTracking;
	blobTracking = new BlobTracking;

	/* Glochidia Counting Algorithm */
	GlochidiaCouting* glochidiaCouting;
	glochidiaCouting = new GlochidiaCouting;

	std::cout << "Press 'q' to quit..." << std::endl;
	int key = 0;
	//IplImage *frame;
	//while(key != 'q')
	while(capture.read(frame))
	{
		//cv::imshow("Input", frame);
		/*if(!capture.read(frame)) {
				std::cerr << "Unable to read next frame." << std::endl;
				std::cerr << "Exiting..." << std::endl;
				exit(EXIT_FAILURE);
			}*/
		//*cv::Mat img_input;
		glochidiaCouting->setCap(frame);
		//*img_input=frame;
		//*glochidiaCouting->setCap(img_input);
		//glochidiaCouting->setCap(frame);
		

		// bgs->process(...) internally process and show the foreground mask image
		cv::Mat img_mask;
		bgs->process(frame, img_mask);

		if(!img_mask.empty())
		{
		  // Perform blob tracking
		  blobTracking->process(frame, img_mask, img_blob);
		  // Perform glochidia counting
		  glochidiaCouting->setInput(img_blob);
		  glochidiaCouting->setTracks(blobTracking->getTracks());
		  glochidiaCouting->process();
		}

		key = cvWaitKey(1);
	}
	
	file<<"Final Count is:"<<countAB;
	std::cout<<"Final Count is:"<<countAB;
	delete glochidiaCouting;
	delete blobTracking;
	delete bgs;
	file.close();
	MessageBox(NULL, "Done, Check count in total_count.txt", "Note", MB_OK);
	cvDestroyAllWindows();
	capture.release();
	return 0;
}
