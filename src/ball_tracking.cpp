#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/statistics/statistics.h>
#include <pcl/visualization/pcl_plotter.h>
#include <vector>
#include <utility>
#include <math.h>  //for abs()
#include "pcl/filters/crop_box.h"
#include "pcl/filters/impl/crop_box.hpp"

using namespace pcl::visualization;

std::vector <float> hist1(361);
std::vector <float> hist2(361);
Eigen::Vector4f minPoint;
Eigen::Vector4f maxPoint;

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"){}

    typedef int (*fptr_)(pcl::PointXYZHSV);

    int func (pcl::PointXYZHSV p)
    {
      return p.h;
    }

    double ChiSquaredDistance (std::vector <float> &hist1, std::vector <float> &hist2)
    {
      if (hist1.size() != hist2.size())
      {
        std::cout << "both histograms do not have the same number of bins" << std::endl;
        return 0;
        PCL_INFO ("[HistogramStatistics::ChiSquaredDistance] : both histograms do not have the same number of bins\n");
      }
      else
      {
        double d = 0;
        int M = 361; int N = 361;
        int counter = 0;
        for (int i = 0; i <= 360; i++)
        {
          if((hist1[i]+hist2[i]) != 0)
          {
            /*
            1/(MN) SUM_i[((Mni - Nmi)^2)/(mi+ni)].
            M and N are the total number of entries in each histogram, mi is the number of entries in bin i of histogram M and ni is the number of entries in bin i of histogram N. 
            */
            d = d + std::pow(M*hist1[i]-N*hist2[i],2)/(hist1[i]+hist2[i]);
            counter++;
          }
        }
        return d/(M*N);
      }
    }

    void setCropBox (Eigen::Vector4f &minPoint, Eigen::Vector4f &maxPoint)
    {
      minPoint[0]= -1;  // define minimum point x
      minPoint[1]= -1;  // define minimum point y
      minPoint[2]= 0;   // define minimum point z
      maxPoint[0]= 1;   // define max point x
      maxPoint[1]= 1;   // define max point y
      maxPoint[2]= 0.7; // define max point z
      return;
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZHSV>);

      pcl::CropBox<pcl::PointXYZRGBA> cropFilter;
      cropFilter.setInputCloud (cloud);
      cropFilter.setMin (minPoint);
      cropFilter.setMax (maxPoint);
      cropFilter.filter (*cloud_filtered);

      PointCloudXYZRGBAtoXYZHSV (*cloud_filtered, *cloud_result);

      pcl::HistogramStatistics <pcl::PointXYZHSV> obj (0, 360, 361, false, true);
      obj.computeHue (*cloud_result, hist2);

      double d = ChiSquaredDistance(hist1, hist2);
      hist1 = hist2;
      hist2.clear ();

      std::cout << "ChiSquaredDistance: " << d << std::endl;

      if (!viewer.wasStopped ())
        viewer.showCloud (cloud_filtered);
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }

      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
 };

int main ()
{
  SimpleOpenNIViewer v;
  v.setCropBox (minPoint, maxPoint);
  v.run ();
  return 0;
}
