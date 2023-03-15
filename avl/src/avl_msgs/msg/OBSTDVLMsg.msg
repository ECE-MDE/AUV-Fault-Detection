#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a Springnav INS obstdvl message containing information about
#          vehicle status information.
#
# AUTHOR:  cong
#===============================================================================

uint8 groupid           # Group ID to associate observations that
                          #are batch processed together
uint8 obmsgtype       # Bit 7 (MSB) = is beam
                        # Bit 6 = is water track
                        # Bits 0-5 = DVL message type:
                        # 0 = Generic PD4
                        # 1 = Generic PD5
                        # 2 = Generic PD0
                        # 8 = LinkQuest PD4
                        # 19 = Syrinx ASONDV
float32 sv       # Sound speed used by Kalman filter
float32 res1     # Residual, Vx or Beam 1 in m/s
float32 res2     # Residual, Vy or Beam 2 in m/s
float32 res3     # Residual, Vz or Beam 3 in m/s
float32 res4     # Residual, Beam 4
