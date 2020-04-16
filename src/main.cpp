#include <iostream>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <sys/time.h>
#include <iostream>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

// ADDS SOME NOISE TO ORIGINAL CLOUD
void addNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    typedef boost::mt19937 RNGType;
    RNGType rng;
    double noise_std = 0.009;
    double noise_mean = 0.0;
    boost::uniform_int<> one_to_six( 0, 6 );
    boost::normal_distribution<> nd (noise_mean, noise_std);

    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > nor_dis (rng, nd);
    boost::variate_generator< RNGType, boost::uniform_int<> > uni_dist(rng, one_to_six);

    /// ADD NOISE
    for ( int i = 0; i < cloud->points.size(); i++ )
      {
        double n  = nor_dis();
        cloud->points[i].x += n;
        cloud->points[i].y += n;
        //cloud->points[i].z += n;

      }

}

// GUIDED FILTER TO SMOOTH NOISY POINT CLOUD
void guidedFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, double epsilon){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    kdtree.setEpsilon(epsilon);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZ searchPoint = cloud->points[i];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3)
      {

        pcl::PointCloud<pcl::PointXYZ>::Ptr neigbors(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::MatrixXd neighbors_as_matrix(3, pointIdxRadiusSearch.size());

        for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
        {
          neigbors->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
          neighbors_as_matrix(0, j) = cloud->points[pointIdxRadiusSearch[j]].x;
          neighbors_as_matrix(1, j) = cloud->points[pointIdxRadiusSearch[j]].y;
          neighbors_as_matrix(2, j) = cloud->points[pointIdxRadiusSearch[j]].z;
        }

        Eigen::Vector3d mean;
        mean = neighbors_as_matrix.rowwise().mean();
        neighbors_as_matrix.transposeInPlace();
        Eigen::MatrixXd centered = neighbors_as_matrix.rowwise() - neighbors_as_matrix.colwise().mean();
        Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(neighbors_as_matrix.rows() - 1);

        Eigen::MatrixXd e = (cov + epsilon * Eigen::MatrixXd::Identity(3, 3));
        e = e.inverse();

        Eigen::MatrixXd A = cov * e;
        Eigen::MatrixXd b = mean - A * mean;

        Eigen::Vector3d searchPointEigenType;
        searchPointEigenType[0] = searchPoint.x;
        searchPointEigenType[1] = searchPoint.y;
        searchPointEigenType[2] = searchPoint.z;

        searchPointEigenType = A * searchPointEigenType + b;

        searchPoint.x = searchPointEigenType[0];
        searchPoint.y = searchPointEigenType[1];
        searchPoint.z = searchPointEigenType[2];
        cloud->points[i] = searchPoint;
      }
    }

    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);

}


int
  main (int argc, char** argv)
{

  std::cout << "Starting To filter Out...";
  if(argc < 4){
      PCL_ERROR ("%d Arguments were provided but 4 arguments are required! \n "
                 "example usage  is; \n ./smooth_pcl ../data/bunny_original.pcd ../data/bunny_noisy.pcd ../data/bunny_smoothed.pcd",argc);
      return(-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


  std::string raw_input_file = argv[1];
  std::string noisy_output_file = argv[2];
  std::string smoothed_output_file = argv[3];

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (raw_input_file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read %s \n",raw_input_file.c_str());
    return (-1);
  }

  addNoise(cloud);

  /// SAVE NOISY CLOUD
  if (pcl::io::savePCDFileASCII (noisy_output_file, *cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't write to file %s \n",noisy_output_file.c_str());
      return (-1);
  }

  float radius = 0.01;
  float epsilon = 0.1;
  guidedFilter(cloud,radius,epsilon);
  guidedFilter(cloud,radius,epsilon);
  guidedFilter(cloud,radius,epsilon);


  /// SAVE Smoothed CLOUD
  if (pcl::io::savePCDFileASCII (smoothed_output_file, *cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't write to file %s \n",smoothed_output_file.c_str());
      return (-1);
  }
  return (0);
}
