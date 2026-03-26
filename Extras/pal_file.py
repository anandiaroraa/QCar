import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped

class QCar():
    """Class for performing basic QCarIO"""
    # Car constants
    WHEEL_RADIUS = 0.0342 # front/rear wheel radius in m
    ENCODER_COUNTS_PER_REV = 720.0 # counts per revolution
    WHEEL_BASE = 0.256 # front to rear wheel distance in m
    WHEEL_TRACK = 0.17 # left to right wheel distance in m
    PIN_TO_SPUR_RATIO = (13.0*19.0) / (70.0*37.0)
        # (diff_pinion*pinion) / (spur*diff_spur)
    CPS_TO_MPS = (1/(ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
        * PIN_TO_SPUR_RATIO * 2*np.pi * WHEEL_RADIUS)


    # Write channels
    WRITE_PWM_CHANNELS = np.array([0], dtype=np.int32)
    WRITE_OTHER_CHANNELS = np.array(
        [1000, 11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003],
        dtype=np.int32
    )

    # Read channels
    READ_ANALOG_CHANNELS = np.array([5, 6], dtype=np.int32)
    READ_ENCODER_CHANNELS = np.array([0], dtype=np.uint32)
    READ_OTHER_CHANNELS = np.array(
        [3000, 3001, 3002, 4000, 4001, 4002, 14000],
        dtype=np.int32
    )


    def __init__(
            self,
            id=0,
            readMode=0,
            frequency=500,
            pwmLimit=0.3,
            steeringBias=0
        ):
        """ Configure and initialize the QCar.

        readMode:
            0 = immediate I/O,
            1 = task based I/O

        id: board identifier id number for virtual use only
        frequency: sampling frequency (used when readMode = 1)
        pwmLimit: maximum (and absolute minimum) command for writing to motors
        steeringBias: steering bias to add to steering command internally
        """

        # Read buffers (internal)
        self.readAnalogBuffer = np.zeros(2, dtype=np.float64)
        self.readEncoderBuffer = np.zeros(1, dtype=np.int32)
        self.readOtherBuffer = np.zeros(7, dtype=np.float64)

        # Read buffers (external)
        self.motorCurrent = np.zeros(2, dtype=np.float64)
        self.batteryVoltage = np.zeros(2, dtype=np.float64)
        self.motorEncoder = np.zeros(1, dtype=np.int32)
        self.motorTach = np.zeros(1, dtype=np.float64)
        self.accelerometer = np.zeros(3, dtype=np.float64)
        self.gyroscope = np.zeros(3, dtype=np.float64)

        # write buffer channels:
        self.writePWMBuffer = np.zeros(1, dtype=np.float64)
        self.writeOtherBuffer = np.zeros(9, dtype=np.float64)

        self.card = HIL()
        self.hardware = IS_PHYSICAL_QCAR
        self.readMode = readMode
        self.io_task_running = False
        self.pwmLimit = pwmLimit
        self._id = str(id)
        self.steeringBias = steeringBias
        self.frequency = frequency

        try:
            if self.hardware:
                boardIdentifier = "0"
            else:
                boardIdentifier = (
                    self._id+"@tcpip://localhost:18960?nagle='off'"
                )

            # Open the Card
            self.card.open("qcar", boardIdentifier)

            if self.card.is_valid():

                # Set PWM mode (duty cycle)
                self.card.set_pwm_mode(
                    np.array([0], dtype=np.uint32),
                    1,
                    np.array([PWMMode.DUTY_CYCLE], dtype=np.int32)
                )

                # Set PWM frequency
                self.card.set_pwm_frequency(
                    np.array([0], dtype=np.uint32),
                    1,
                    np.array([60e6/4096], dtype=np.float64)
                )

                # Set Motor coast to 0
                self.card.write_digital(
                    np.array([40], dtype=np.uint32),
                    1,
                    np.zeros(1, dtype=np.float64)
                )

                # Set board-specific options
                boardOptionsString = ("steer_bias=" + str(self.steeringBias)
                    + ";motor_limit=" + str(self.pwmLimit) + ';')
                self.card.set_card_specific_options(
                    boardOptionsString,
                    MAX_STRING_LENGTH
                )

                # Set Encoder Properties
                self.card.set_encoder_quadrature_mode(
                    self.READ_ENCODER_CHANNELS,
                    len(self.READ_ENCODER_CHANNELS),
                    np.array([4],
                    dtype=np.uint32)
                )
                self.card.set_encoder_filter_frequency(
                    self.READ_ENCODER_CHANNELS,
                    len(self.READ_ENCODER_CHANNELS),
                    np.array([60e6/1],
                    dtype=np.uint32)
                )
                self.card.set_encoder_counts(
                    self.READ_ENCODER_CHANNELS,
                    len(self.READ_ENCODER_CHANNELS),
                    np.zeros(1, dtype=np.int32)
                )

                if self.hardware and self.readMode == 1:
                    self._create_io_task()

                print('QCar configured successfully.')

        except HILError as h:
            print(h.get_error_message())


    def _create_io_task(self):
        # Define reading task
        self.readTask = self.card.task_create_reader(
            int(self.frequency*2),
            self.READ_ANALOG_CHANNELS,
            len(self.READ_ANALOG_CHANNELS),
            self.READ_ENCODER_CHANNELS,
            len(self.READ_ENCODER_CHANNELS),
            None,
            0,
            self.READ_OTHER_CHANNELS,
            len(self.READ_OTHER_CHANNELS)
        )

        # Set buffer overflow mode depending on
        # whether its for hardware or virtual QCar
        if self.hardware:
            self.card.task_set_buffer_overflow_mode(
                self.readTask,
                BufferOverflowMode.OVERWRITE_ON_OVERFLOW
            )
        else:
            self.card.task_set_buffer_overflow_mode(
                self.readTask,
                BufferOverflowMode.WAIT_ON_OVERFLOW
            )

        # Start the reading task
        self.card.task_start(
            self.readTask,
            Clock.HARDWARE_CLOCK_0,
            self.frequency,
            2**32-1
        )
        self.io_task_running = True


    def terminate(self):
        # This function terminates the QCar card after setting
        # final values for throttle, steering and LEDs.
        # Also terminates the task reader.

        try:
            # write 0 PWM command, 0 steering, and turn off all LEDs
            self.card.write(
                None,
                0,
                self.WRITE_PWM_CHANNELS,
                len(self.WRITE_PWM_CHANNELS),
                None,
                0,
                self.WRITE_OTHER_CHANNELS,
                len(self.WRITE_OTHER_CHANNELS),
                None,
                np.zeros(len(self.WRITE_PWM_CHANNELS), dtype=np.float64),
                None,
                np.zeros(len(self.WRITE_OTHER_CHANNELS), dtype=np.float64)
            )

            # if using Task based I/O, stop the readTask.
            if self.readMode:
                self.card.task_stop(self.readTask)
            self.card.close()

        except HILError as h:
            print(h.get_error_message())

    def read_write_std(self, throttle, steering, LEDs=None):
        """ Read and write standard IO signals for the QCar

        Use this to write throttle, steering and LED commands, as well as
            update buffers for battery voltage, motor current,
            motor encoder counts, motor tach speed, and IMU data.

        throttle - this method will saturate based on the pwmLimit.

        steering - this method will saturate from -0.6 rad to 0.6 rad

        LEDs - a numpy string of 8 values

        Updates the following 6 buffers: motorCurrent, batteryVoltage,
            accelerometer, gyroscope, motorEncoder, motorTach

        """
        self.write(throttle, steering, LEDs)
        self.read()


    def read(self):
        if not (self.hardware or self.io_task_running) and self.readMode:
            self._create_io_task()

        try:
            # if using task based I/O, use the read task
            if self.readMode == 1:
                self.card.task_read(
                    self.readTask,
                    1,
                    self.readAnalogBuffer,
                    self.readEncoderBuffer,
                    None,
                    self.readOtherBuffer
                )
            else: # use immediate I/O
                self.card.read(
                    self.READ_ANALOG_CHANNELS,
                    len(self.READ_ANALOG_CHANNELS),
                    self.READ_ENCODER_CHANNELS,
                    len(self.READ_ENCODER_CHANNELS),
                    None,
                    0,
                    self.READ_OTHER_CHANNELS,
                    len(self.READ_OTHER_CHANNELS),
                    self.readAnalogBuffer,
                    self.readEncoderBuffer,
                    None,
                    self.readOtherBuffer
                )
        except HILError as h:
            print(h.get_error_message())
        finally:
            # update external read buffers
            self.motorCurrent = self.readAnalogBuffer[0]
            self.batteryVoltage = self.readAnalogBuffer[1]
            self.gyroscope = self.readOtherBuffer[0:3]
            self.accelerometer = self.readOtherBuffer[3:6]
            self.motorEncoder = self.readEncoderBuffer
            self.motorTach = self.readOtherBuffer[-1] * QCar.CPS_TO_MPS


    def write(self, throttle, steering, LEDs=None):
        if not (self.hardware or self.io_task_running) and self.readMode:
            self._create_io_task()

        self.writePWMBuffer = -np.clip(throttle, -self.pwmLimit, self.pwmLimit)
        self.writeOtherBuffer[0] = -np.clip(steering, -0.6, 0.6)
        if LEDs is not None:
            self.writeOtherBuffer[1:9] = LEDs

        try:
            self.card.write(
                None,
                0,
                self.WRITE_PWM_CHANNELS,
                len(self.WRITE_PWM_CHANNELS),
                None,
                0,
                self.WRITE_OTHER_CHANNELS,
                len(self.WRITE_OTHER_CHANNELS),
                None,
                self.writePWMBuffer,
                None,
                self.writeOtherBuffer
            )
        except HILError as h:
            print(h.get_error_message())

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.terminate()

class QCarMonitor:
    def __init__(self):
        rospy.init_node('qcar_monitor', anonymous=True)
        
        # Subscribe to the QCar navigation topic
        self.subscription = rospy.Subscriber(
            '/qcar/mux/ackermann_cmd_mux/input/navigation',
            AckermannDriveStamped,
            self.listener_callback,
            queue_size=10
        )
        
        rospy.loginfo("QCar Monitor started. Listening to /qcar/mux/ackermann_cmd_mux/input/navigation")
        rospy.loginfo("Waiting for messages...")
    
    def listener_callback(self, msg):
        """Callback function that processes received messages"""
        speed = msg.drive.speed
        steering = msg.drive.steering_angle
        
        # Determine direction
        if speed > 0:
            direction = "FORWARD"
        elif speed < 0:
            direction = "BACKWARD"
        else:
            direction = "STOPPED"
        
        # Determine steering
        if abs(steering) < 0.01:
            turn = "STRAIGHT"
        elif steering > 0:
            turn = f"LEFT ({steering:.2f} rad)"
        else:
            turn = f"RIGHT ({steering:.2f} rad)"
        
        rospy.loginfo(f"Command: {direction} @ {abs(speed):.2f} m/s | Steering: {turn}")

def main():
    try:
        monitor = QCarMonitor()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()