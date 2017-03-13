#pragma once

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "cvblob/cvblob.h"

class BlobTracking
{
private:
  bool firstTime;
  int minArea;
  int maxArea;
  
  bool debugTrack;
  bool debugBlob;
  bool showBlobMask;
  bool showOutput;

  cvb::CvTracks tracks;
  void saveConfig();
  void loadConfig();

public:
  BlobTracking();
  ~BlobTracking();

  void process(const cv::Mat &img_input, const cv::Mat &img_mask, cv::Mat &img_output);
  const cvb::CvTracks getTracks();
};

