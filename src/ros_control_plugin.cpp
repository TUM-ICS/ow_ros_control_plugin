/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_ros_control_plugin/ros_control_plugin.h>

using namespace std;
using namespace hardware_interface;

namespace ow_ros_control_plugin
{

  OwRosControlPlugin::OwRosControlPlugin() : simulation_(false)
  {
    ROS_INFO_STREAM("Calling ow_ros_control_plugin constructor.");
  }

  OwRosControlPlugin::~OwRosControlPlugin()
  {
    ROS_INFO_STREAM("Calling ow_ros_control_plugin destructor.");
  }

  bool OwRosControlPlugin::init(PositionJointInterface *pos_iface,
                                ForceTorqueSensorInterface *ft_iface,
                                ImuSensorInterface *imu_iface,
                                ros::NodeHandle &root_nh,
                                ros::NodeHandle &controller_nh)
  {
    //--------------------------------------------------------------------------
    // Hardware

    // Load Hardware interfaces
    if (!initJoints(pos_iface, controller_nh) ||
        !initForceTorqueSensors(ft_iface, controller_nh) ||
        !initImuSensors(imu_iface, controller_nh))
    {
      ROS_ERROR_STREAM("Failed to initialize harware interface '"
                       << internal::demangledTypeName(*this) << "'");
      return false;
    }

    //--------------------------------------------------------------------------
    // Openwalker

    // load the global openwalker parameters
    ow::Parameter parameter;
    parameter.add<bool>("simulation", true);
    parameter.add<ow::Scalar>("publish_rate", 200.0);
    parameter.add<ow::Scalar>("loop_rate", 200.0);
    parameter.add<ow::Scalar>("hip_height", 0.8);
    parameter.add<ow::Scalar>("step_time", 1.2);
    parameter.add<ow::Scalar>("t_double_support", 0.3);
    parameter.add<ow::Scalar>("t_single_support", 0.9);
    parameter.add<ow::Vector3>("kp_imu_offset", ow::Vector3::Zero());
    parameter.add<std::string>("robot_description", "", false);
    if (!parameter.load("/open_walker"))
    {
      ROS_ERROR("OwRosControlPlugin::init: error loading parameters");
      return false;
    }

    // read
    parameter.get("simulation", simulation_);

    // Init the hardware
    hw_interface_.reset(new ow_hw_interface::HwInterface(
      parameter.get<ow::Scalar>("loop_rate"),
      joint_handles_, 
      ft_handles_, 
      ft_sensor_offsets_,
      imu_handle_, 
      X_imu_base_));

    if (!hw_interface_->initRequest(parameter, controller_nh) != 0)
    {
      ROS_ERROR_STREAM("Failed to initialize robot.");
      return false;
    }

    //controller_.reset(new ow_controller::HomeingController(*hw_interface_.get()));
    //controller_.reset(new ow_controller::BalancingController(*hw_interface_.get()));
    controller_.reset(new ow_controller::WalkingController(*hw_interface_.get()));

    if (!controller_->initRequest(parameter, controller_nh) != 0)
    {
      ROS_ERROR_STREAM("Failed to initialize controller.");
      return false;
    }

    // Get current state to initialize controller.
    hw_interface_->update();
    hw_interface_->updateCommand(hw_interface_->lastJointStateCommand().pos());

    // print state
    if (simulation_)
    {
      ROS_INFO_STREAM("Plugin " << controller_nh.getNamespace()
                                << " configured for Gazebo Simulation");
    }
    else
    {
      ROS_INFO_STREAM("Plugin " << controller_nh.getNamespace()
                                << " configured for REAL ROBOT");
    }
    return true;
  }

  void OwRosControlPlugin::starting(const ros::Time &time)
  {
    time_prev_ = time;
    controller_->startRequest(*hw_interface_.get(), time);
    ROS_WARN_STREAM("OwRosControlPlugin::starting: " << time.toSec());
  }

  void OwRosControlPlugin::update(const ros::Time &time,
                                  const ros::Duration &period)
  {
    ros::Duration dt = time - time_prev_;
    time_prev_ = time;

    if (simulation_)
    {
      if ((time.nsec % 5000000) == 0)
      {
        updateOWController(time, dt);
      }
    }
    else
    {
      updateOWController(time, dt);
    }
    ros::spinOnce();
  }

  void OwRosControlPlugin::updateOWController(const ros::Time &time, const ros::Duration &dt)
  {
    // update the robot state
    hw_interface_->update();

    // update the controller
    ow::JointState q;
    q = controller_->updateRequest(*hw_interface_.get(), time, dt);

    // write the resulting command to robot
    hw_interface_->updateCommand(q.pos());

    // send the command to real robot
    for (int ii = 0; ii < q.pos().rows(); ii++)
    {
      joint_handles_.at(ii).setCommand(q.pos()(ii));
    }
  }

  void OwRosControlPlugin::stopping(const ros::Time &time)
  {
    controller_->stopRequest(*hw_interface_.get(), time);
    ROS_INFO_STREAM("OwRosControlPlugin::stopping: " << time.toSec());
  }

  bool OwRosControlPlugin::initRequest(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle &root_nh,
                                       ros::NodeHandle &controller_nh,
                                       ClaimedResources &claimed_resources)
  {
    // Get a pointer to the joint position control interface
    PositionJointInterface *pos_iface = robot_hw->get<PositionJointInterface>();
    if (!pos_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW"
                " class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // Get a pointer to the force-torque sensor interface
    ForceTorqueSensorInterface *ft_iface =
        robot_hw->get<ForceTorqueSensorInterface>();

    if (!ft_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW"
                " class.",
                internal::demangledTypeName<ForceTorqueSensorInterface>().c_str());
      return false;
    }

    // Get a pointer to the IMU sensor interface
    ImuSensorInterface *imu_iface = robot_hw->get<ImuSensorInterface>();
    if (!imu_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW"
                " class.",
                internal::demangledTypeName<ImuSensorInterface>().c_str());
      return false;
    }

    // initalize the everything
    pos_iface->clearClaims();
    if (!init(pos_iface,
              ft_iface,
              imu_iface,
              root_nh,
              controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      std::cerr << "FAILED LOADING OPEN WALKER" << std::endl;
      return false;
    }

    claimed_resources.push_back(
        InterfaceResources(
            internal::demangledTypeName<PositionJointInterface>(),
            pos_iface->getClaims()));
    pos_iface->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }

  bool OwRosControlPlugin::initJoints(PositionJointInterface *pos_iface,
                                      ros::NodeHandle &nh)
  {
    // Get joint names from the parameter server
    using namespace XmlRpc;

    XmlRpcValue joint_names;

    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("No joints given (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }

    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed joint specification (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }

    // Populate container of joint handles
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];

      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Array of joint names should contain all strings"
                         " (namespace:"
                         << nh.getNamespace() << ").");
        return false;
      }
      const string joint_name = static_cast<string>(name_value);

      // Get a joint handle
      try
      {
        joint_handles_.push_back(pos_iface->getHandle(joint_name));
        ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '"
                                         << getHardwareInterfaceType() << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '"
                                                  << getHardwareInterfaceType() << "'");
        return false;
      }
    }

    return true;
  }

  bool OwRosControlPlugin::initForceTorqueSensors(
      ForceTorqueSensorInterface *ft_iface, ros::NodeHandle &nh)
  {
    // Get ft_sensor names from the parameter server
    using namespace XmlRpc;
    XmlRpcValue ft_sensor_names;
    if (!nh.getParam("ft_sensors", ft_sensor_names))
    {
      ROS_ERROR_STREAM("No ft_sensors given (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }
    if (ft_sensor_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed ft_sensor specification (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }

    // Populate container of force torque sensors
    for (int i = 0; i < ft_sensor_names.size(); ++i)
    {
      XmlRpcValue &name_value = ft_sensor_names[i];

      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Array of ft_sensor names should contain all strings"
                         " (namespace:"
                         << nh.getNamespace() << ").");
        return false;
      }
      const string ft_sensor_name = static_cast<string>(name_value);

      // Get a joint handle
      try
      {
        ft_handles_.push_back(ft_iface->getHandle(ft_sensor_name));
        ROS_DEBUG_STREAM("Found ft_sensor '" << ft_sensor_name << "' in '"
                                             << getHardwareInterfaceType() << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find ft_sensor '" << ft_sensor_name
                                                      << "' in '" << getHardwareInterfaceType() << "'");
        return false;
      }
    }

    // Load calibration offsets.
    XmlRpcValue ft_sensor_offsets;
    ft_sensor_offsets_.clear();
    if (!nh.getParam("ft_sensors_calib", ft_sensor_offsets))
    {
      ROS_ERROR_STREAM("No ft_sensors_calib given (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }

    if (ft_sensor_offsets.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed ft_sensor_offset specification (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }
    for (int i = 0; i < ft_sensor_offsets.size(); ++i)
    {
      XmlRpcValue &offset_value = ft_sensor_offsets[i];

      if (offset_value.getType() != XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("Array of ft_sensor_offset should contain float "
                         "vectors of size 6."
                         " (namespace:"
                         << nh.getNamespace() << ").");
        return false;
      }

      if (offset_value.size() != 6)
        ROS_ERROR("ft_sensors_calib [%d] has no 6 elements", i);
      else
      {
        ow::Wrench W;
        for (int j = 0; j < offset_value.size(); j++)
        {
          W(j) = offset_value[j];
        }
        ft_sensor_offsets_.push_back(W);
      }
    }
    return true;
  }

  bool OwRosControlPlugin::initImuSensors(ImuSensorInterface *imu_iface,
                                          ros::NodeHandle &nh)
  {
    // Base IMU
    string imu_name;
    if (!nh.getParam("imu_sensor", imu_name))
    {
      ROS_ERROR_STREAM("No imu_sensor given (namespace:"
                       << nh.getNamespace() << ").");
      return false;
    }
    try
    {
      imu_handle_ = imu_iface->getHandle(imu_name);
      if (!imu_handle_.getOrientation())
      {
        ROS_ERROR_STREAM("IMU sensor '" << imu_name
                                        << "' does not provide orientation readings");
        return false;
      }

      // load imu data
      ow::Vector3 rpy_imu_b = ow::Vector3::Zero();
      ow::LinearPosition x_imu_b = ow::LinearPosition::Zero();
      if (!ow::load(nh.getNamespace() + "/imu_sensor_calib/rpy_imu_base", rpy_imu_b))
      {
        ROS_WARN("OwRosControlPlugin: no imu_sensor_calib/rpy_imu_b found in %s",
                 nh.getNamespace().c_str());
      }
      if (!ow::load(nh.getNamespace() + "/imu_sensor_calib/t_imu_base", x_imu_b))
      {
        ROS_WARN("OwRosControlPlugin: no imu_sensor_calib/t_imu_base found in %s",
                 nh.getNamespace().c_str());
      }
      X_imu_base_.orientation() = ow::Rotation3::RPY(rpy_imu_b);
      X_imu_base_.position() = x_imu_b;
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Could not find IMU sensor '"
                       << imu_name << "' in '" << internal::demangledTypeName(*imu_iface) << "'");
      return false;
    }
    return true;
  }

} // namespace ow_ros_control_plugin
