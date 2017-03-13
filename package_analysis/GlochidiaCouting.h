#pragma once

#include <iostream>
#include <string>
#include <cv.hpp>
#include "/usr/local/include/opencv2/highgui/highgui.hpp"

#include <opencv2/imgproc.hpp>
#include "../package_tracking/cvblob/cvblob.h"
extern   long countAB;
enum LaneOrientation
{
  LO_NONE       = 0,
  LO_HORIZONTAL = 1,
  LO_VERTICAL   = 2
};

enum GlochidiaPosition
{
  VP_NONE = 0,
  VP_A  = 1,
  VP_B  = 2
};

class GlochidiaCouting
{
private:
  bool firstTime;
  bool showOutput;
  int key;
  cv::Mat img_input;
  cvb::CvTracks tracks;
  std::map<cvb::CvID, std::vector<CvPoint2D64f> > points;
  LaneOrientation laneOrientation;
  std::map<cvb::CvID, GlochidiaPosition> positions;

  long countBA;
  int img_w;
  int img_h;
  int showAB;

public:
  GlochidiaCouting();
  ~GlochidiaCouting();

  void setInput(const cv::Mat &i);
  void setCap(const cv::Mat &i);
  void setTracks(const cvb::CvTracks &t);
  void process();

private:
  GlochidiaPosition getGlochidiaPosition(const CvPoint2D64f centroid);

  void saveConfig();
  void loadConfig();
};
