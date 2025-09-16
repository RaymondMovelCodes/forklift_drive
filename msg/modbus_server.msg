# topic_server ros message
#data1
Header header
int16 driveSpeed            # Drive speed in r/min, positive: forward, negative: backward
int8  liftControl           # Lift  up/down control, 0: stop, 1: up, 2: down
int16 steerAngle            # Steering angle in 0.01°, positive: right, negative: left
uint8 emergencyStop         # Emergency stop button, 0 when pressed
uint8 bumper                # bumper triggered
uint8 buttonStop            # Stop button, 1 when pressed
uint8 buttonStart           # Start button, 1 when pressed
uint8 leftForkTipAvoid      # Left fork tip obstacle avoidance, 1 when detected
uint8 rightForkTipAvoid     # Right fork tip obstacle avoidance, 1 when detected
uint8 forkUpLimitStatus     # Fork lower limit, 1 when detected
uint8 forkDownLimitStatus   # Fork upper limit, 1 when detected
uint8 cargoStatus			# Cargo detection, 0: no Cargo, 1: Cargo on the forklift
uint8 chargeStatus          # Charging receiver, 1 when detected
uint8 mp3Channel            # MP3 channel, 1-7
uint8 buzzer                # Check buzzer, 1: on, 0: off
uint8 isAuto                # Manual/Auto, 0: manual, 1: auto

#data2
uint8 leftObstacleLidarTrigger1       # Check left obstatic lidar detection area 1 trigger, 1 when detected
uint8 leftObstacleLidarTrigger2       # Check left obstatic lidar detection area 2 trigger, 1 when detected
uint8 leftObstacleLidarTrigger3       # Check left obstatic lidar detection area 3 trigger, 1 when detected
uint8 rightObstacleLidarTrigger1      # Check right obstatic lidar detection area 1 trigger, 1 when detected
uint8 rightObstacleLidarTrigger2      # Check right obstatic lidar detection area 2 trigger, 1 when detected
uint8 rightObstacleLidarTrigger3      # Check right obstatic lidar detection area 3 trigger, 1 when detected
uint8 driveMotorErrorCode        # drive motor fault codes
uint8 steeringMotorErrorCode     # steering motor fault codes
uint8 liftMotorErrorCode     	 # lift motor fault codes
uint8 hardwareStatus       # Check hardware Status, 0:offline, 1:online
						   # bit0->Curtis controller status
						   # bit1->battery status
						   # bit2->left ultrasonic radar status
						   # bit3->right ultrasonic radar status
uint8 batteryPctg          # Battery SOC, 0~100% 
uint8 batteryVal           # Battery voltage, unit: V
float32 batteryCurrent     # Battery Current, unit: *0.1 Ah

#data3
uint16 leftMiddleUltrasonicRadarValue   # Left middle ultrasonic radar data, mm
uint16 rightMiddleUltrasonicRadarValue 	# Left middle ultrasonic radar data
int32 dirveMotorEncoder         		# motor encoder
uint16 leftUpUltrasonicRadarValue 		# Left upper ultrasonic radar data, mm
uint16 leftDownUltrasonicRadarValue  	# Left down ultrasonic radar data, mm
uint16 rightUpUltrasonicRadarValue 		# right upper ultrasonic radar data, mm
uint16 rightDownUltrasonicRadarValue 	# right down ultrasonic radar data, mm
uint8 wingsStatus        				# Current position of wings, 
										# bit0->rise status
										# bit1->fall status
