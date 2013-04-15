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

using namespace pcl::visualization;

class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    int func(pcl::PointXYZHSV p)
    {
      return p.h;
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZHSV>);

      pcl::PassThrough<pcl::PointXYZRGBA> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (0.0, 5.0);
      pass.filter (*cloud_filtered);
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (0.0, 5.0);
      pass.filter (*cloud_filtered);
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 0.9);
      pass.filter (*cloud_filtered);

      PointCloudXYZRGBAtoXYZHSV (*cloud_filtered, *cloud_result);

      int n_points = cloud_result->points.size ();
      int mean_h = 0;

/*
      if (n_points != 0)
      {
        for (int i = 0; i < n_points; i++)
        {
          mean_h = mean_h + cloud_result->points[i].h;
        }
        mean_h = mean_h/n_points;
        std::cout << "Mean H(SV) value: " << mean_h << std::endl;
      }
*/

      std::vector <float> histogram;
      pcl::HistogramStatistics <pcl::PointCloud<pcl::PointXYZHSV> > obj (0, 359, 360);
      obj.setFunctionPointer (func);
      obj.compute (*cloud_result, *histogram);
      std::cout << "Bin 10: " << histogram[10] << std::endl;
      histogram.clear ();

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

/*
  //defining a plotter
  pcl::visualization::PCLPlotter* plotter = new PCLPlotter("Color histogram");
  std::vector<double> data(10);
  data[0] = 4;
  data[1] = 2;
  data[2] = 3;
  data[3] = 8;
  data[4] = 4;
  data[5] = 6;
  data[6] = 7;
  data[7] = 9;
  data[8] = 1;
  data[9] = 2; 

  plotter->addHistogramData (data,10); //number of bins are 10
  //display the plot
  plotter->plot ();
  plotter->clearPlots ();
*/

  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
