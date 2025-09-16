# topic_client ros message

Header header
int16 driveSpeed            # Drive speed in r/min, positive: forward, negative: backward
int8  liftControl           # Lift  up/down control, 0: stop, 1: up, 2: down
int16 steerAngle            # Steering angle in 0.01°, positive: right, negative: left
uint8 lightRed              # Tricolor light red, 1: on, 0: off
uint8 lightGreen            # Tricolor light green, 1: on, 0: off
uint8 lightYellow           # Tricolor light yellow, 1: on, 0: off
uint8 leftObstacleLidarMap  # Left obstacle avoidance lidar map, range: 1-8
uint8 rightObstacleLidarMap # right obstacle avoidance lidar map, range: 1-8
uint8 sensorRecharge        # Auto charging IR sensor, 1: on, 0: off
uint8 sensorCharge          # Charging contactor, 1: on, 0: off
uint8 mp3Channel            # MP3 channel, 1-7
uint8 buzzer                # Tricolor light buzzer, 1: on, 0: off
bool isDebug                # Print debug info
uint8 wingsMotorControl     # Wings motor control, 0: stop, 1: open, 2: close
uint8 warningLightControl   # Warning light control, 0: off, 1: on

