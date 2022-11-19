/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "abb_driver/abb_utils.h"
#include "industrial_robot_client/joint_trajectory_downloader.h"
#include "industrial_utils/param_utils.h"

using industrial_robot_client::joint_trajectory_downloader::JointTrajectoryDownloader;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

class ABB_JointTrajectoryDownloader : public JointTrajectoryDownloader
{
  using JointTrajectoryDownloader::init;  // so base-class init() stays visible

  bool J23_coupled_;

public:
  bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION)
  {
    if (!JointTrajectoryDownloader::init(default_ip, default_port))  // call base-class init()
      return false;

    if (ros::param::has("J23_coupled"))
      ros::param::get("J23_coupled", this->J23_coupled_);
    else
      J23_coupled_ = false;

    return true;
  }

  bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in, trajectory_msgs::JointTrajectoryPoint* pt_out)
  {
    // correct for parallel linkage effects, if desired
    //   - use POSITIVE factor for joint->motor correction
    abb::utils::linkage_transform(pt_in, pt_out, J23_coupled_ ? +1:0 );

    return true;
  }

  bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
  {
    *rbt_velocity = 0;  // unused by ABB driver
    return true;
  }
};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "motion_interface");

  // launch the default JointTrajectoryDownloader connection/handlers
  ABB_JointTrajectoryDownloader motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
