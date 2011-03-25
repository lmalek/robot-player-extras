#include <cmath>
#include <unistd.h>
#include <cstdlib>
#include <float.h>
#include <iostream>

#include "te.h"

#ifndef SIGN
#define SIGN(x) (((x) == 0) ? 0 : (((x) > 0) ? 1 : -1))
#endif

extern PlayerTime *GlobalTime;

// Initialization function
Driver* 
TE_Init(ConfigFile* cf, int section) 
{
  return ((Driver*) (new TE( cf, section)));
} 

// a driver registration function
void TE_Register(DriverTable* table)
{
  char nazwa[]="te";
  table->AddDriver(nazwa,  TE_Init);
  return;
} 

////////////////////////////////////////////////////////////////////////////////
// Constructor
TE::TE( ConfigFile* cf, int section)
  : Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE)
{
  this->dist_eps = cf->ReadTupleLength(section, "goal_tol", 0, 0.5);
  this->ang_eps = cf->ReadTupleAngle(section, "goal_tol", 1, DTOR(10.0));

  this->vx_max = cf->ReadFloat(section, "max_speed", 0.3);
  this->va_max = cf->ReadFloat(section, "max_rot",  DTOR(45.0));
  this->vx_min = cf->ReadFloat(section, "min_speed",  0.05);
  this->va_min = cf->ReadFloat(section, "min_rot", DTOR(10.0));

  this->min_dist = cf->ReadLength(section, "min_dist", 0.3);
  this->warning_dist_on = cf->ReadLength(section, "warning_dist_on", 0.4);
  this->warning_dist_off = cf->ReadLength(section, "warning_dist_off", 0.41);
  this->obs_dist = cf->ReadLength(section, "obs_dist", 0.7);
  
  this->laser_min_angle = cf->ReadFloat(section, "laser_min_angle",
					-DTOR(90));
  this->laser_max_angle = cf->ReadFloat(section, "laser_max_angle",
					DTOR(90));
  														 														 
  														 

  this->k_a = cf->ReadFloat(section, "k_a", 1);

  this->odom = NULL;
  if (cf->ReadDeviceAddr(&this->odom_addr, section, "requires",
                         PLAYER_POSITION2D_CODE, -1, "output") != 0)
    {
      this->SetError(-1);    
      return;
    }

  this->localize = NULL;
  if (cf->ReadDeviceAddr(&this->localize_addr, section, "requires",
                         PLAYER_POSITION2D_CODE, -1, "input") != 0)
    {
      this->SetError(-1);    
      return;
    }

  this->laser = NULL;
  memset(&this->laser_addr,0,sizeof(player_devaddr_t));
  cf->ReadDeviceAddr(&this->laser_addr, section, "requires",
                     PLAYER_LASER_CODE, -1, NULL);
  if(this->laser_addr.interf)
    {
      this->laser_buffer = cf->ReadInt(section, "laser_buffer", 10);
    }

  
  return;
}


TE::~TE() 
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int TE::Setup() 
{
  // Initialise the underlying position device.
  if (this->SetupOdom() != 0)
    return -1;
  
  this->active_goal = false;
  
  // Initialise the laser.
  if (this->laser_addr.interf && this->SetupLaser() != 0)
    return -1;

  this->stall = false;
  this->turning_in_place = false;
  this->last_odom_pose.px = this->last_odom_pose.py = this->last_odom_pose.pa = FLT_MAX;
  
  this->obstacle = false;

  this->waiting = false;

  // Start the driver thread.
  this->StartThread();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
int TE::Shutdown() 
{
  // Stop the driver thread.
  this->StopThread();

  // Stop the laser
  if(this->laser)
    this->ShutdownLaser();

  // Stop the odom device.
  this->ShutdownOdom();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the underlying odom device.
int TE::SetupOdom() 
{
  // Setup the output position device
  if(!(this->odom = deviceTable->GetDevice(this->odom_addr)))
    {
      PLAYER_ERROR("unable to locate suitable output position device");
      return -1;
    }
  if(this->odom->Subscribe(this->InQueue) != 0)
    {
      PLAYER_ERROR("unable to subscribe to output position device");
      return -1;
    }

  // Setup the input position device
  if(!(this->localize = deviceTable->GetDevice(this->localize_addr)))
    {
      PLAYER_ERROR("unable to locate suitable input position device");
      return -1;
    }

  if(this->localize->Subscribe(this->InQueue) != 0)
    {
      PLAYER_ERROR("unable to subscribe to input position device");
      return -1;
    }

  // Get the odometry geometry
  Message* msg;
  if(!(msg = this->odom->Request(this->InQueue,
                                 PLAYER_MSGTYPE_REQ,
                                 PLAYER_POSITION2D_REQ_GET_GEOM,
                                 NULL, 0, NULL,false)) ||
     (msg->GetHeader()->size != sizeof(player_position2d_geom_t)))
    {
      PLAYER_ERROR("failed to get geometry of underlying position device");
      if(msg)
	delete msg;
      return(-1);
    }
  player_position2d_geom_t* geom = (player_position2d_geom_t*)msg->GetPayload();

  this->robot_geom = *geom;
  printf("robot geom: %.3f %.3f %.3f %.3f %.3f\n",
         this->robot_geom.size.sl,
         this->robot_geom.size.sw,
         this->robot_geom.pose.px,
         this->robot_geom.pose.py,
         RTOD(this->robot_geom.pose.pa));
         

  delete msg;

  memset(&this->odom_pose, 0, sizeof(player_pose_t));
  memset(&this->odom_vel, 0, sizeof(player_pose_t));
  this->odom_stall = false;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the underlying odom device.
int TE::ShutdownOdom() 
{
  // Stop the robot before unsubscribing
  this->PutPositionCmd(0.0,0.0);
  this->odom->Unsubscribe(this->InQueue);
  this->localize->Unsubscribe(this->InQueue);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the laser
int TE::SetupLaser() 
{
  if(!(this->laser = deviceTable->GetDevice(this->laser_addr)))
    {
      PLAYER_ERROR("unable to locate suitable laser device");
      return -1;
    }
  if (this->laser->Subscribe(this->InQueue) != 0)
    {
      PLAYER_ERROR("unable to subscribe to laser device");
      return -1;
    }

  player_laser_geom_t* cfg;
  Message* msg;

  // Get the laser pose
  if(!(msg = this->laser->Request(this->InQueue, 
                                  PLAYER_MSGTYPE_REQ,
                                  PLAYER_LASER_REQ_GET_GEOM,
                                  NULL, 0, NULL,false)))
    {
      PLAYER_ERROR("failed to get laser geometry");
      return(-1);
    }
  
  // Store the laser pose
  cfg = (player_laser_geom_t*)msg->GetPayload();
  this->laser_pose = cfg->pose;
  delete msg;

  this->num_laser_scans = 0;

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shut down the laser
int TE::ShutdownLaser() 
{
  this->laser->Unsubscribe(this->InQueue);
  return 0;
}


// Send a command to the motors
void
TE::PutPositionCmd(double vx, double va)
{
  player_position2d_cmd_vel_t cmd;

  memset(&cmd,0,sizeof(player_position2d_cmd_vel_t));

  cmd.vel.px = vx;
  cmd.vel.pa = va;
  cmd.state = 1;
  // cmd.type = 0;
  //printf("sending: %.3f %.3f\n", vx, RTOD(va));

  this->odom->PutMsg(this->InQueue,
                     PLAYER_MSGTYPE_CMD,
                     PLAYER_POSITION2D_CMD_VEL,
                     (void*)&cmd,sizeof(cmd),NULL);
}

void
TE::ProcessInputOdom(player_msghdr_t* hdr, player_position2d_data_t* data)
{
  this->odom_pose = data->pos;

  player_msghdr_t newhdr = *hdr;
  newhdr.addr = this->device_addr;
  player_position2d_data_t newdata;

  newdata.pos = data->pos;
  newdata.vel = this->odom_vel;
  if(data->stall)
    {
      this->PutPositionCmd(0.0,0.0);
      this->waiting = true;
      newdata.stall = 0;
    }
  else
    {
      newdata.stall = 0;
      this->waiting = false;
    }

  // this->stall indicates that we're stuck (either TE threw an emergency 
  // stop or it was failing to make progress).  Set the stall flag to let
  // whoever's listening that we've given up.
  if(this->stall)
    newdata.stall = 1;

  this->Publish(NULL, &newhdr, &newdata);
}

void
TE::ProcessOutputOdom(player_msghdr_t* hdr, player_position2d_data_t* data)
{
  this->odom_vel = data->vel;
  // Stage's stall flag seems to be broken
  //this->odom_stall = data->stall;
  this->odom_stall = false;
}

void
TE::ProcessLaser(player_msghdr_t* hdr, player_laser_data_t* scan)
{
  double r, b, db;
  double dmin = this->obs_dist;
  this->obstacle = false;
  this->beta=M_PI/2;

  db = scan->resolution;

  for(unsigned int i=0;i<scan->ranges_count;i++)
    {
      b = scan->min_angle + (i * db);
      r = scan->ranges[i];
      if (b >= laser_min_angle && b <= laser_max_angle){
	if (r<dmin){
	  this->obstacle = true;
	  dmin=r;
	  this->beta = b;
    	}	
      }
    }
  nearObstDist = dmin;
  //std::cout<<"Beta = "<<this->beta<<std::endl;
  if (dmin<=this->min_dist){
    std::cout<<"STOPED"<<std::endl;
    this->waiting = true;
    this->PutPositionCmd(0, 0);
    this->stall=true;
  }
  if (dmin>this->min_dist){
    //this->waiting = false;
    //this->warning = true;	
  }
  /*
    if (dmin<=this->warning_dist_on){
    std::cout<<"TE: warning ON"<<std::endl;
    this->warning = true;
    warningEscapeGoal.px = this->obs_dist;
    warningEscapeGoal.py = 0;
    warningEscapeGoal.pa = -this->odom_pose.pa;
    }
    if (dmin>this->warning_dist_off){
    this->warning = false;
    std::cout<<"TE: warning OFF"<<std::endl;
    }
  */
  //std::cout<<"TE: ProcessLaser beta = "<< this->beta<<std::endl;
}

void 
TE::ProcessCommand(player_msghdr_t* hdr, player_position2d_cmd_vel_t* cmd)
{
  if(!cmd->state)
    {
      this->PutPositionCmd(0.0,0.0);
      this->active_goal = false;
    }
 
  else 
    {
      PLAYER_MSG2(2, "Stopped by velocity command (%.3f %.3f)",
		  cmd->vel.px, RTOD(cmd->vel.pa));
      // TODO: bylo wylaczone w ostatniej wersji, ale wtedy nie zatrzymuje sie
      this->PutPositionCmd(cmd->vel.px, cmd->vel.pa);
      this->active_goal = false;
    }
  if(cmd->vel.px < 0)
    this->dir = -1;
  else
    this->dir = 1;
}

void 
TE::ProcessCommand(player_msghdr_t* hdr, player_position2d_cmd_pos_t* cmd)
{
  PLAYER_MSG3(2, "New goal: (%.3f %.3f %.3f)",
              cmd->pos.px, 
              cmd->pos.py, 
              RTOD(cmd->pos.pa));
  // position control;  cache the goal and we'll process it in the main
  // loop.  also cache vel.px, which tells us whether to go forward or
  // backward to get to the goal.
  this->goal = cmd->pos;

  this->active_goal = true;
  this->turning_in_place = false;
  this->stall = false;
  GlobalTime->GetTimeDouble(&this->translate_start_time);
  this->last_odom_pose = this->odom_pose;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int TE::ProcessMessage(MessageQueue* resp_queue, 
                       player_msghdr * hdr, 
                       void * data)
{
  // Is it new odometry data?
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
                           PLAYER_POSITION2D_DATA_STATE, 
                           this->odom_addr))
    {
      this->ProcessOutputOdom(hdr, (player_position2d_data_t*)data);

      // In case the input and output are the same device
      if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
			       PLAYER_POSITION2D_DATA_STATE, 
			       this->localize_addr))
	this->ProcessInputOdom(hdr, (player_position2d_data_t*)data);

      return(0);
    }
  // Is it new localization data?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
				PLAYER_POSITION2D_DATA_STATE, 
				this->localize_addr))
    {
      this->ProcessInputOdom(hdr, (player_position2d_data_t*)data);
      return(0);
    }
  // Is it a new laser scan?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
                                PLAYER_LASER_DATA_SCAN, 
                                this->laser_addr))
    {
      this->ProcessLaser(hdr, (player_laser_data_t*)data);
      return(0);
    }
  // Is it a new goal?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
				PLAYER_POSITION2D_CMD_POS,
                                this->device_addr))
    {
      this->ProcessCommand(hdr, (player_position2d_cmd_pos_t*)data);
      return 0;
    }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
                                PLAYER_POSITION2D_CMD_VEL, 
                                this->device_addr))
    {
      this->ProcessCommand(hdr, (player_position2d_cmd_vel_t*)data);
      return 0;
    }
  // Is it a request for the underlying device?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, -1, this->device_addr))
    {
      // Pass the request on to the underlying position device and wait for
      // the reply.
      Message* msg;

      if(!(msg = this->odom->Request(this->InQueue,
				     hdr->type,
				     hdr->subtype,
				     (void*)data, 
				     hdr->size, 
				     &hdr->timestamp)))
	{
	  PLAYER_WARN1("failed to forward config request with subtype: %d\n",
		       hdr->subtype);
	  return(-1);
	}

      player_msghdr_t* rephdr = msg->GetHeader();
      void* repdata = msg->GetPayload();
      // Copy in our address and forward the response
      rephdr->addr = this->device_addr;
      this->Publish(resp_queue, rephdr, repdata);
      delete msg;
      return(0);
    }
  else
    return -1;
}

double
TE::Threshold(double v, double v_min, double v_max)
{
  if(v == 0.0)
    return(v);
  else if(v > 0.0)
    {
      v = MIN(v, v_max);
      v = MAX(v, v_min);
      return(v);
    }
  else
    {
      v = MAX(v, -v_max);
      v = MIN(v, -v_min);
      return(v);
    }
}

void
TE::Main()
{
  double g_dx, g_da;
  double alpha, theta, phi;
  double vx, va;
  player_pose_t relativeGoal;
  player_pose_t currGoal;

  // Fill in the TE's parameter structure

  this->current_dir = 1;

  for(;;)
    {
      usleep(200000); // 200 ms delay

      pthread_testcancel();

      // this->laser_obstacles get updated by this
      // call
      this->ProcessMessages();

      // are we waiting for a stall to clear?
      if(this->waiting){
	this->PutPositionCmd(0, 0);
	this->stall = true;
	std::cout<<"waiting"<<std::endl;
	continue;
      }
   
      // do we have a goal?
      if(!this->active_goal){
	//std::cout<<"no goal"<<std::endl;
	continue;
      }
      
      // wzgledne polozenie celu
      relativeGoal.px = this->goal.px-this->odom_pose.px;
      relativeGoal.py = this->goal.py-this->odom_pose.py;
      relativeGoal.pa = this->goal.pa;

      // angle from 0 to the goal (theta)
      theta = atan2(relativeGoal.py, relativeGoal.px);
      // diff betwean robot orientation angle (psi) and goal vector (theta)
      alpha = angle_diff(theta,this->odom_pose.pa);
      g_dx = hypot(relativeGoal.px, relativeGoal.py);

      
      if (this->obstacle && g_dx>this->dist_eps) {
	//PLAYER_MSG0(1, "TE: obstacle avoidance");
	if (fabs(this->beta) > this->ang_eps)
	  phi = angle_diff(fabs(this->beta) / this->beta * M_PI/2, 
			   angle_diff(this->beta, alpha));
	else
	  phi = angle_diff(M_PI/2, angle_diff(this->beta,alpha));
    	currGoal.px = cos(phi) * relativeGoal.px +
	  sin(phi) * relativeGoal.py;
    	currGoal.py = -sin(phi) * relativeGoal.px + 
	  cos(phi) * relativeGoal.py;
    	currGoal.pa = relativeGoal.pa;
      }
      else
    	currGoal = relativeGoal;
  
      /*  	
		if (this->warning) {
		currGoal.px = cos(M_PI) * warningEscapeGoal.px +
		sin(M_PI) * warningEscapeGoal.py;
		currGoal.py = -sin(M_PI) * warningEscapeGoal.px + 
		cos(M_PI) * warningEscapeGoal.py;
		currGoal.pa = warningEscapeGoal.pa;
		}
      */

      // angle from 0 to the goal (theta)
      theta = atan2(currGoal.py, currGoal.px);
      // diff betwean robot orientation angle (psi) and goal vector (theta)
      alpha = angle_diff(theta,this->odom_pose.pa);
      // are we at the goal?
      g_dx = hypot(currGoal.px, currGoal.py);
      g_da = this->angle_diff(currGoal.pa, this->odom_pose.pa);
    
      //std::cout<<"TE: beta = "<<(this->beta*180/M_PI)<<std::endl;
      //    std::cout<<"TE: phi = "<<(phi*180/M_PI)<<std::endl;
    

      if((g_dx < this->dist_eps)) { // jestesmy bliko celu
	std::cout<<"TE: blisko celu"<<std::endl;
	if (fabs(g_da) < this->ang_eps) { // z wlasciwa orientacja
	  this->active_goal = false;
	  this->PutPositionCmd(0.0,0.0);
	  std::cout<<"TE: At goal"<<std::endl;
	  continue;
	}
	else { // trzeba poprawić orientację po dojechaniu do celu
	  std::cout<<"TE: poprawa orientacji"<<std::endl;
	  vx = 0;
	  va = this->k_a*this->va_max * tanh(10*g_da);
	}
      }
      else {
	// sterowanie
	vx = this->vx_max * tanh(fabs(g_dx)) * fabs(cos(alpha));
	va = this->k_a * this->va_max * tanh(alpha);
      }
      //    if (this->obstacle)
      //    	vx = vx*0.75;
      /*
	if (this->warning){
    	vx = 0.1*vx;
	}
      */
      std::cout<<"nearObstDist = "<<nearObstDist<<std::endl;
      if (nearObstDist <= obs_dist) {
	vx=vx*(nearObstDist-min_dist)/(obs_dist-min_dist);
      }
      if (nearObstDist <= min_dist) {vx=0; va=0;}

      //    std::cout<<"vx_max = "<<vx_max<<" va_max = "<<va_max<<std::endl;
      //   std::cout<<"alpha = "<<alpha*180/M_PI<<std::endl;    
      //vx = this->Threshold(vx, this->vx_min, this->vx_max);
      //va = this->Threshold(va, this->va_min, this->va_max);	
      std::cout<<"vx = "<<vx<<" va = "<<va<<std::endl;
      this->PutPositionCmd(vx, va);
    }
}

// computes the signed minimum difference between the two angles.
double
TE::angle_diff(double a, double b)
{
  double d1, d2; 
  a = NORMALIZE(a);
  b = NORMALIZE(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.
#if 1
/* need the extern to avoid C++ name-mangling  */
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    puts("TE driver initializing");
    TE_Register(table);
    puts("TE initialization done");
    return(0);
  }
}
#endif
