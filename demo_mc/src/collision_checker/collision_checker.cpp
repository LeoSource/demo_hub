#include <pinocchio/fwd.hpp>
#include <iostream>
#include <memory>
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/collision_object.h"
#include "hpp/fcl/collision_utility.h"
#include "hpp/fcl/data_types.h"
#include "hpp/fcl/distance.h"
#include "hpp/fcl/mesh_loader/loader.h"
#include "hpp/fcl/shape/convex.h"
#include "hpp/fcl/shape/geometric_shapes.h"


int main(int, char**){
  std::cout << "Hello, from collision checker!\n";
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::GeometryModel geom_model;
  pinocchio::GeometryData geom_data;
  std::string urdf_path = "/home/zy/Documents/a2w_t3d5/robot_visualization/src/robot_model/urdf/raise_a2_w_t3d5/model.urdf";
  std::string pkg_path = "/home/zy/Documents/a2w_t3d5/robot_visualization/src/";
  std::vector<std::string> pkg_dirs = {pkg_path};
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, geom_model, pkg_dirs);
  for (int i = 0; i < geom_model.geometryObjects.size(); i++) {
    std::cout << "Geometry object " << i << ": " << geom_model.geometryObjects.at(i).name << std::endl;
    // if (auto fcl_geom = std::dynamic_pointer_cast<hpp::fcl::CollisionGeometry>(geom_model.geometryObjects[i].geometry))
      // std::cout<<"success"<<std::endl;
    // hpp::fcl::OBJECT_TYPE obj_type = geom_model.geometryObjects[i].geometry->getObjectType();
    // if (obj_type == hpp::fcl::OBJECT_TYPE::OT_BVH) {
    //   std::string mesh_file = geom_model.geometryObjects[i].meshPath;
    //   geom_model.geometryObjects[i].geometry = LoadConvexMesh(mesh_file);
    // }
    // std::cout << "geometry object type: " << hpp::fcl::get_object_type_name(geom_model.geometryObjects[i].geometry->getObjectType()) << std::endl;
    // std::cout << "geometry node type: " << hpp::fcl::get_node_type_name(geom_model.geometryObjects[i].geometry->getNodeType()) << std::endl;
  }
}