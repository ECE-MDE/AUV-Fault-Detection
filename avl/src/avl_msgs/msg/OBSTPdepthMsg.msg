#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a Springnav INS obstpdepth message containing information about
#          vehicle status information.
#
# AUTHOR:  cong
#===============================================================================

uint8 obmsgtype       # 0 – Keller
                        # 1 – PSONDEP
                        # 2 – DigiQuartz (m)
                        # 3 – DigiQuartz (psi)
                        # 4 – DigiQuartz (kPa)
                        # 5 – $__DPT
                        # 6 – PRDDIGIQO
                        # 7 – Winson
                        # 8 – Valeport SVX2 (DBAR)
                        # 9 XDEPTH

float32 resdepth        # Residual depth, in m
