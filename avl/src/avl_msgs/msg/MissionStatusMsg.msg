#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a message used to relay the status of a mission.
#===============================================================================

# Mission mode that the mission manager is currently in
uint8 mode

# FSD mission state
uint8 fsd_mission_state

 # Action number currently being executed from the FSD mission queue
uint16 fsd_current_action

 # Total number of actions in the FSD mission queue
uint16 fsd_total_actions

# Percent completion of current FSD action
float32 fsd_action_percent

# BSD mission state
uint8 bsd_mission_state

# Action number currently being executed from the BSD mission queue
uint16 bsd_current_action

# Total number of actions in the BSD mission queue
uint16 bsd_total_actions

# Percent completion of current BSD action
float32 bsd_action_percent
