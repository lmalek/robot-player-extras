/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * $Id: flashsip.h,v 1.7.2.1 2007/04/24 22:20:20 gerkey Exp $
 *
 * part of the P2OS parser.  methods for filling and parsing server
 * information packets (FLASHSIPs)
 */
#ifndef _FLASHSIP_H
#define _FLASHSIP_H

#include <climits>

#include "flash.h"

typedef struct ArmJoint
{
	char speed;
	unsigned char home;
	unsigned char min;
	unsigned char centre;
	unsigned char max;
	unsigned char ticksPer90;
} ArmJoint;

class FLASHSIP
{
 private:
  int PositionChange( unsigned short, unsigned short );
  int param_idx; // index of our robot's data in the parameter table

 public:
  // these values are returned in every standard FLASHSIP
  bool lwstall, rwstall;
  unsigned char status, battery, sonarreadings, analog, digin, digout;
  unsigned short ptu, compass, timer, rawxpos;
  unsigned short rawypos, frontbumpers, rearbumpers;
  short angle, lvel, rvel, control;
  unsigned short sonars[PLAYER_SONAR_MAX_SAMPLES];
  int xpos, ypos;
  int x_offset,y_offset,angle_offset;

  // these values are returned in a CMUcam serial string extended FLASHSIP
  // (in host byte-order)
  unsigned short blobmx, blobmy;	// Centroid
  unsigned short blobx1, blobx2, bloby1, bloby2;	// Bounding box
  unsigned short blobarea, blobconf;	// Area and confidence
  unsigned int	 blobcolor;

  // This value is filled by ParseGyro()
  int32_t gyro_rate;

  // This information comes from the ARMpac and ARMINFOpac packets
  bool armPowerOn, armConnected;
  bool armJointMoving[6];
  unsigned char armJointPos[6];
  double armJointPosRads[6];
  unsigned char armJointTargetPos[6];
  char *armVersionString;
  unsigned char armNumJoints;
  ArmJoint *armJoints;

  // Need this value to calculate approx position of lift when in between up and down
  double lastLiftPos;

  /* returns 0 if Parsed correctly otherwise 1 */
  void Parse( unsigned char *buffer );
  void ParseSERAUX( unsigned char *buffer );
  void ParseGyro(unsigned char* buffer);
  void ParseArm (unsigned char *buffer);
  void ParseArmInfo (unsigned char *buffer);
  void Print();
  void PrintSonars();
  void PrintArm ();
  void PrintArmInfo ();
  void Fill(player_flash_data_t* data);

  FLASHSIP(int idx)
  {
    param_idx = idx;
    for(int i=0;i<ARRAYSIZE(sonars);i++)
      sonars[i] = 0;

    xpos = INT_MAX;
    ypos = INT_MAX;

    // intialise some of the internal values
    blobmx = blobmy = blobx1 = blobx2 = bloby1 = bloby2 = blobarea = blobconf = blobcolor = 0;
    armPowerOn = armConnected = false;
    armVersionString = NULL;
    armJoints = NULL;
    armNumJoints = 0;
    for (int i = 0; i < 6; ++i)
    {
      armJointMoving[i] = false;
      armJointPos[i] = 0;
      armJointPosRads[i] = 0;
      armJointTargetPos[i] = 0;
    }
  }
};

#endif
