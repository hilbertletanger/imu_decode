Header header           # packet header
bool isLessInfo
uint8 reserved
# lessInfo start
#yaw
float32 VehDynYawRateHSC2
float32 VehDynYawRateVHSC2
#shift
int32 TrShftLvrPos_h1HSC2
int32 TrShftLvrPosV_h1HSC2
#spped
float32 VehSpdAvgDrvnHSC2
int32 VehSpdAvgDrvnVHSC2
float32 VehSpdAvgNonDrvnHSC2
int32 VehSpdAvgNonDrvnVHSC2
#wheel
float32 StrgWhlAngHSC2
int32 StrgWhlAngVHSC2
# lessInfo end

# more info start
int16 nCounter
# INERTIAL
int64 iTimeStamp
float64 fLatitude
float64 fLongitude
float32 fAltitude
float32 fAccX
float32 fAccY
float32 fAccZ
float32 fAngRateX
float32 fAngRateY
float32 fAngRateZ
float32 fVelNorth
float32 fVelWest
float32 fVelUp
float32 fHeading
float32 fPitch
float32 fRoll
int8 nNavStatus
#uint8 nWiperStatus
#float32 fDrnvelL
#float32 fNonDrnvelL
#float32 fDrnvelR
#float32 fNonDrnvelR
#float32 dTrailerAngle

# VCU_VEHICLE_INFO
int64 vTimeStamp
#float32 fSteeringAngleVel
float32 fSteeringAngle
float32 fSpeed
float32 fYawRate
float32 fFrontLeftWheelSpeed
float32 fFrontRightWheelSpeed
float32 fRearLeftWheelSpeed
float32 fRearRightWheelSpeed
uint8 nShifterPosition
uint8 nLeftDirectionLamp
uint8 nRightDirectionLamp
uint8 nMainBeamLamp
uint8 nDippedBeamLamp
uint8 nWiperState
float32 fLateralAccel
float32 fLongituAccel
int16 nLeftDrivenWheelPulseCounters
int16 nRightDrivenWheelPulseCounters
int16 nLeftNonDrivenWheelPulseCounters
int16 nRightNonDrivenWheelPulseCounters
uint8 nDriveMode

# more info end
