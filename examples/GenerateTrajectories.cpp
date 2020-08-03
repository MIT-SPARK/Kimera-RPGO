/*
Read g2o file and separate multirobot trajectories (output in csv)
author: Yun Chang
*/

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

using namespace gtsam;

int main(int argc, char* argv[]) {
  GraphAndValues graphNValues;
  std::string input_file = argv[1];
  std::string output_folder = argv[2];
  graphNValues = gtsam::load3D(input_file);
  Values all_values = *graphNValues.second;
  std::map<char, Values> trajectories;
  // Sepearate trajectories of different robots
  for (size_t i = 0; i < all_values.keys().size(); i++) {
    Symbol key = Symbol(all_values.keys()[i]);
    char prefix = key.chr();
    if (trajectories.find(prefix) == trajectories.end()) {
      trajectories[prefix] = Values();
    }
    trajectories[prefix].insert(key, all_values.at(key));
  }
  // Write to csv
  for (std::pair<char, Values> traj : trajectories) {
    std::string file_name = output_folder + "/robot_" + traj.first + ".csv";
    std::ofstream output;
    output.open(file_name);
    output << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w "
              "[], q_RS_x [], q_RS_y [], q_RS_z [], \n";

    // Generate
    for (size_t i = 0; i < traj.second.keys().size(); i++) {
      Key key = traj.second.keys()[i];
      Pose3 pose = traj.second.at<Pose3>(key);
      Point3 position = pose.translation();
      Quaternion quat = pose.rotation().toQuaternion();

      output << "0 , " << position.x() << "," << position.y() << ","
             << position.z() << "," << quat.w() << "," << quat.x() << ","
             << quat.y() << "," << quat.z() << ",\n";
    }
    output.close();
  }
}
