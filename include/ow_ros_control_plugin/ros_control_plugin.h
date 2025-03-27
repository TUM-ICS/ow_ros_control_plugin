#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <ow_core/types.h>

#include <ow_hw_interface/hw_interface.h>
#include <ow_controller/homeing_controller.h>
#include <ow_controller/walking_controller.h>
#include <ow_controller/balancing_controller.h>

using namespace std;
using namespace hardware_interface;

/*!
 * \brief Open Walker ros control plugin namespace. These plugin libraries
 * implement the interface to ros control.
 */
namespace ow_ros_control_plugin
{

  /*!
 * \brief The OwRosControlPlugin class
 * 
 * This class is the bridge between Open Walker and ros_control. This is an
 * example of a plugin which claims the hardware resources and connects them
 * with the Open Walker hardware interface.
 *
 * This plugin also instanciates and calls the init and update functions of an
 * Open Walker controller.
 */
  class OwRosControlPlugin
      : public controller_interface::ControllerBase
  {
  public:
    typedef hardware_interface::ForceTorqueSensorHandle FTHandle;
    typedef hardware_interface::ImuSensorHandle ImuHandle;
    typedef hardware_interface::JointHandle JointHandle;

  private:
    bool simulation_; //!< True if the plugin running in the Gazebo simulator.

    /*!
   * \brief Abstraction of the robot hardware.
   */
    std::unique_ptr<ow_hw_interface::HwInterface> hw_interface_;

    /*!
    * \brief Open Walker Controller.
    */
    std::unique_ptr<ow::ControllerBase> controller_;

    /*!
    * \brief Calibration offsets for the FT sensors.
    */
    std::vector<ow::Wrench> ft_sensor_offsets_;

    /*!
    * \brief CartesianPosition of the IMU sensor wrt the base link.
    */
    ow::CartesianPosition X_imu_base_;

    // Hardware interfaces

    /*!
    * \brief Hardware interface for IMU sensor.
    */
    ImuHandle imu_handle_;

    /*!
    * \brief Hardware interface for the Force-Torque sensors.
    */
    std::vector<FTHandle> ft_handles_;

    /*!
    * \brief Hardware interface for the joints..
    */
    std::vector<JointHandle> joint_handles_;

    ros::Time time_prev_;

  public:
    /*!
    * \brief Default constructor.
    */
    OwRosControlPlugin();

    /**
    * @brief Destroy the Ow Ros Control Plugin object
    * 
    */
    ~OwRosControlPlugin();

  public:

    bool initRequest(hardware_interface::RobotHW *robot_hw,
                     ros::NodeHandle &root_nh,
                     ros::NodeHandle &controller_nh,
                     ClaimedResources &claimed_resources);

    void starting(const ros::Time &time);

    void update(const ros::Time &time, const ros::Duration &period);

    void stopping(const ros::Time &time);

    std::string getHardwareInterfaceType() const
    {
      return hardware_interface::internal::
          demangledTypeName<hardware_interface::PositionJointInterface>();
    }

  private:

    /*!
    * \brief init
    * \param pos_iface
    * \param ft_iface
    * \param imu_iface
    * \param controller_nh
    * \return
    */
    bool init(PositionJointInterface *pos_iface,
              ForceTorqueSensorInterface *ft_iface,
              ImuSensorInterface *imu_iface,
              ros::NodeHandle &root_nh,
              ros::NodeHandle &controller_nh);

    bool initJoints(PositionJointInterface *pos_iface,
                    ros::NodeHandle &controller_nh);

    bool initForceTorqueSensors(ForceTorqueSensorInterface *ft_iface,
                                ros::NodeHandle &controller_nh);

    bool initImuSensors(ImuSensorInterface *imu_iface,
                        ros::NodeHandle &controller_nh);

    void updateOWController(const ros::Time &time, const ros::Duration &dt);
    
  };

  PLUGINLIB_EXPORT_CLASS(ow_ros_control_plugin::OwRosControlPlugin,
                         controller_interface::ControllerBase)

} // namespace ow_ros_control_plugin
