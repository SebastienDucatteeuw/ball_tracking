#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>

class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

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

      if (n_points != 0)
      {
        for (int i = 0; i < n_points; i++)
        {
          mean_h = mean_h + cloud_result->points[i].h;
        }
        mean_h = mean_h/n_points;
        std::cout << "Mean H(SV) value: " << mean_h << std::endl;
      }

      if (!viewer.wasStopped())
        viewer.showCloud (cloud_filtered);

    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

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
  v.run ();
  return 0;
}
