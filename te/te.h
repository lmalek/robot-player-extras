/** @brief Tangential Escape
    
    This driver implements the TANGENTIAL ESCAPE 

    The driver itself supports the @interface_position2d interface.  Send
    @ref PLAYER_POSITION2D_CMD_POS commands to set the goal pose.  The driver
    also accepts @ref PLAYER_POSITION2D_CMD_VEL commands, simply passing them
    through to the underlying output device.
    

    @par Compile-time dependencies
    
    - none
    
    @par Provides
    
    - @ref interface_position2d
    
    @par Requires
    
    - "input" @ref interface_position2d : source of pose and velocity information
    - "output" @ref interface_position2d : sink for velocity commands to control the robot
    - @ref interface_laser : the laser to read from
    
    @par Configuration requests
    
    - all @ref interface_position2d requests are passed through to the
    underlying "output" @ref interface_position2d device.
    
    @par Configuration file options
    
    - goal_tol (tuple: [length angle])
    - Default: [0.5 10.0] (m deg)
    - Respectively, translational and rotational goal tolerance.  When the
    robot is within these bounds of the current target pose, it will
    be stopped.
    
    - max_speed (tuple: [length/sec angle/sec])
    - Default: [0.3 45.0] (m/s deg/s)
    - Respectively, maximum absolute translational and rotational velocities
    to be used in commanding the robot.
    
    - min_speed (tuple: [length/sec angle/sec])
    - Default: [0.05 10.0] (m/s deg/s)
    - Respectively, minimum absolute non-zero translational and rotational
    velocities to be used in commanding the robot.
    
    - obs_dist (length)
    - Default: 0.7 m
    - Distance at which obstacle avoidance begins
    
    - min_dist (length)
    - Default: 0.2 m
    - Distance at which obstacle avoidance begins
    
    - laser_buffer (integer)
    - Default: 10          
    - How many recent laser scans to consider in the local navigation.
    
    @par Example
    
    @verbatim
    driver
    (
    name "te"
    provides ["position2d:1"]
    requires ["output:::position2d:0" "input:::position2d:0" "laser:0"]
    
    max_speed [0.3 30.0]
    min_speed [0.1 10.0]
    goal_tol [0.3 15.0]
    
    min_dist 0.4
    
    laser_buffer 10
    )
    @endverbatim
    
    @author Łukasz Małek (implementation)
*/

#ifndef ROBOT_PLAYER_EXTRAS_TE_H
#define ROBOT_PLAYER_EXTRAS_TE_H

#include <cmath>
#include <unistd.h>
#include <cstdlib>
#include <float.h>
#include <iostream>

#include <libplayercore/playercore.h>

class TE : public Driver 
{
 public:
  /** Constructor
   */
  TE( ConfigFile* cf, int section);
  
  /** Destructor
   */
  virtual ~TE();
  
  /** Setup routune
   */
  virtual int Setup();
  /** Shutdown routine
   */
  virtual int Shutdown();
  
  /** Process incoming messages from clients
   */ 
  virtual int ProcessMessage(MessageQueue* resp_queue, 
			     player_msghdr * hdr, 
			     void * data);
  /** Main function for device thread.
   */
  virtual void Main();
  
 private:
  bool active_goal;
  int dir;
  player_pose_t goal;
  player_pose_t last_odom_pose;
  player_pose_t odom_pose;
  player_pose_t odom_vel;
  bool odom_stall;
  int current_dir;
  
  double rotate_start_time;
  double rotate_min_error;
  double rotate_stuck_time;
  
  double translate_start_time;
  double translate_min_error;
  double translate_stuck_time;
  double translate_stuck_dist;
  double translate_stuck_angle;
  bool wait_on_stall;
  bool waiting;
  bool warning;

  bool stall;
  bool turning_in_place;
    
  bool obstacle;
  double nearObstDist;

  int num_laser_scans;
  double laser_min_angle, laser_max_angle;

  double vx_max, va_max;
  double vx_min, va_min;
  double k_a;
  double obs_dist;
  player_position2d_geom_t robot_geom;
  double min_dist;
  double warning_dist_on, warning_dist_off;
  player_pose_t warningEscapeGoal;
    
  double Threshold(double v, double v_min, double v_max);
  int SetupOdom();
  int ShutdownOdom();
  int SetupLaser();
  int ShutdownLaser();

  void ProcessOutputOdom(player_msghdr_t* hdr, player_position2d_data_t* data);
  void ProcessInputOdom(player_msghdr_t* hdr, player_position2d_data_t* data);
  void ProcessLaser(player_msghdr_t* hdr, player_laser_data_t* data);
  void ProcessCommand(player_msghdr_t* hdr, player_position2d_cmd_vel_t* cmd);
  void ProcessCommand(player_msghdr_t* hdr, player_position2d_cmd_pos_t* cmd);
  // Send a command to the motors
  void PutPositionCmd(double vx, double va);

  // Computes the signed minimum difference between the two angles.
  double angle_diff(double a, double b);
    
  // Odometry device info
  Device *odom;
  player_devaddr_t odom_addr;
  Device *localize;
  player_devaddr_t localize_addr;
  double dist_eps;
  double ang_eps;

  // Laser device info
  Device *laser;
  player_devaddr_t laser_addr;
  player_pose_t laser_pose;
  int laser_buffer;

  //algorithm parameters
  double beta, alpha, phi, rho;
};
#endif
