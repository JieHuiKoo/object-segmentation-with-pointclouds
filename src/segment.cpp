#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <sstream>

ros::Publisher pub;
double _max_distance = 0.005;
double _min_percentage = 5;
bool _color_pc_with_error = false;



class ColorMap{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B){
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }


private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};

class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};
std::vector<Color> colors;
double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

sensor_msgs::PointCloud2 apply_voxel_filter(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // Create container for input data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;       // Request for memory allocation on the heap 
  pcl::PCLPointCloud2ConstPtr cloudPTR(cloud);                // for a pointer to a type pcl::PCLPointCloud2

  pcl::PCLPointCloud2 cloud_filtered;                         // Filtered cloud
  
  sensor_msgs::PointCloud2 output;                            // Output cloud

  //Convert from sensor_msgs to pcl
  pcl_conversions::toPCL(*inputCloud, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPTR);
  sor.setLeafSize (0.04, 0.04, 0.04);
  sor.filter(cloud_filtered);

  pcl_conversions::fromPCL(cloud_filtered, output);

  return output;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;       // Request for memory allocation on the heap 
  pcl::PCLPointCloud2ConstPtr cloudPTR(cloud);                // for a pointer to a type pcl::PCLPointCloud2
  
  pcl_conversions::toPCL(*msg, *cloud);
  
  sensor_msgs::PointCloud2 cloud_publish;
  pcl_conversions::fromPCL(*cloud,cloud_publish);
  pub.publish(cloud_publish);
}

void createColors(){
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        for (int i=0;i<20;i++){
            while (r<70 && g < 70 && b < 70){
                r = rand()%(255);
                g = rand()%(255);
                b = rand()%(255);
            }
            Color c(r,g,b);
            r = 0;
            g = 0;
            b = 0;
            colors.push_back(c);
        }
    }

int main(int argc, char **argv)
{
  createColors();

  // Initialise ROS and specify name of node 
  ros::init(argc, argv, "segment");
  
  // Initialise the node handle
  ros::NodeHandle n;

  // Initialise the pub object
  // This pub object will advertise a PointCloud2 sensor_msgs with the topic /segmented_cloud and buffer of 1
  pub = n.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);

  // Subscribe message
  ros::Subscriber sub = n.subscribe("/armCamera/depth_registered/points", 1, cloud_callback);

  ros::spin();
  }
