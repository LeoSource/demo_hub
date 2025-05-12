#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include "hpp/fcl/collision.h"
#include "hpp/fcl/BVH/BVH_model.h"
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/octree.h>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"


#define PINOCCHIO_WITH_HPP_FCL

int main(int, char**){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zy/Downloads/data/biwi_face_database/model1.pcd", *cloud) == -1) {
      PCL_ERROR("Failed to load PCD file\n");
      return -1;
  }
  std::cout << "Loaded " << cloud->size() << " points" << std::endl;

  pinocchio::GeometryObject::CollisionGeometryPtr collision_geometry_ptr;

  std::string urdf_file;
  pinocchio::Model model;
  // pinocchio::urdf::buildModel(urdf_file, model);


  std::vector<hpp::fcl::Vec3f> points;
  std::shared_ptr<hpp::fcl::BVHModel<hpp::fcl::OBBRSS>> cloud_geom(new hpp::fcl::BVHModel<hpp::fcl::OBBRSS>);
  cloud_geom->beginModel();
  for (int i = 0; i < cloud->points.size(); i++) {
    hpp::fcl::Vec3f point;
    point(0) = cloud->points[i].x;
    point(1) = cloud->points[i].y;
    point(2) = cloud->points[i].z;
    cloud_geom->addVertex(point);
    points.push_back(point);
  }
  cloud_geom->endModel();
  cloud_geom->buildConvexHull(true, "Qt");
  std::cout <<"original num_vertices: " << cloud_geom->num_vertices << std::endl;
  std::cout <<"original num_triangles: " << cloud_geom->num_tris << std::endl;
  // collision_geometry_ptr = cloud_geom->convex;
  std::cout << "convex num_vertices: " << cloud_geom->convex->num_points << std::endl;
  std::cout << "convex Volume: " << cloud_geom->convex->computeVolume() << std::endl;
  // std::cout << "collision geometry: " << collision_geometry_ptr->num_points << std::endl;



  double resolution = 0.01;
  auto octree = std::make_shared<hpp::fcl::OcTree>(resolution);
}
