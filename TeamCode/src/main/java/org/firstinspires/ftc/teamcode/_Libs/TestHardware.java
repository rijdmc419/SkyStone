package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class TestHardware implements HardwareDevice {

    // test hardware classes -- useful for testing when no hardware is available.
    // these are primarily intended for use in testing autonomous mode code, but could
    // also be useful for testing tele-operator modes.

    public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }
    public String getDeviceName() { return "AutoLib_TestHardware"; }
    public String getConnectionInfo() { return "connection info unknown"; }
    public int getVersion() { return 0; }
    public void resetDeviceConfigurationForOpMode() {}
    public void close() {}


    // a dummy DcMotor that just logs commands we send to it --
    // useful for testing Motor code when you don't have real hardware handy
    static public class TestMotor extends TestHardware implements DcMotor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this motor
        double mPower;      // current power setting
        DcMotor.RunMode mMode;
        int mTargetPosition;
        int mCurrentPosition;
        boolean mPowerFloat;
        Direction mDirection;
        ZeroPowerBehavior mZeroPowerBehavior;
        int mMaxSpeed;
        MotorConfigurationType mMotorType;

        public TestMotor(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPower = 0.0;
            mMaxSpeed = 0;
            mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            mTargetPosition = 0;
            mCurrentPosition = 0;
            mPowerFloat = false;
            mDirection = Direction.FORWARD;
            mZeroPowerBehavior = ZeroPowerBehavior.FLOAT;
        }

        @Override       // override all the functions of the real DcMotor class that touch hardware
        public void setPower(double power) {
            mPower = power;
            mOpMode.telemetry.addData(mName, " power: " + String.valueOf(mPower));
        }

        public double getPower() {
            return mPower;
        }

        public void close() {
            mOpMode.telemetry.addData(mName, " close();");
        }

        public boolean isBusy() {
            return false;
        }

        public void setPowerFloat() {
            mPowerFloat = true;
            mOpMode.telemetry.addData(mName, " setPowerFloat();");
        }

        public boolean getPowerFloat() {
            return mPowerFloat;
        }

        public void setMaxSpeed(int encoderTicksPerSecond)
        {
            mMaxSpeed = encoderTicksPerSecond;
            mOpMode.telemetry.addData(mName, "maxSpeed: " + String.valueOf(encoderTicksPerSecond));
        }

        public int getMaxSpeed() { return mMaxSpeed; }

        public void setTargetPosition(int position) {
            mTargetPosition = position;
            mOpMode.telemetry.addData(mName, "target: " + String.valueOf(position));
        }
        public int getTargetPosition() {
            return mTargetPosition;
        }

        public int getCurrentPosition() {
            return mTargetPosition;
        }

        public void setMode(DcMotor.RunMode mode) {
            this.mMode = mode;
            mOpMode.telemetry.addData(mName, "run mode: " + String.valueOf(mode));
        }

        public DcMotor.RunMode getMode() {
            return this.mMode;
        }

        public void setDirection(Direction direction)
        {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(direction));
        }

        public Direction getDirection() { return mDirection; }

        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public DcMotorController getController()
        {
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public int getPortNumber()
        {
            return 0;
        }

        public ZeroPowerBehavior getZeroPowerBehavior()
        {
            return mZeroPowerBehavior;
        }

        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior)
        {
            mZeroPowerBehavior = zeroPowerBehavior;
            mOpMode.telemetry.addData(mName, "zeroPowerBehavior: " + String.valueOf(zeroPowerBehavior));
        }

        public String getDeviceName() { return "AutoLib_TestMotor: " + mName; }

        public void setMotorType(MotorConfigurationType motorType) { mMotorType = motorType; }

        public MotorConfigurationType getMotorType() { return mMotorType; }

    }

    // a dummy Servo that just logs commands we send to it --
    // useful for testing Servo code when you don't have real hardware handy
    static public class TestServo extends TestHardware implements Servo {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this servo
        double mPosition;   // current target position
        Direction mDirection;
        double mScaleMin;
        double mScaleMax;

        public TestServo(String name, OpMode opMode) {
            super();     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPosition = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPosition(double position) {
            mPosition = position;
            mOpMode.telemetry.addData(mName, " position: " + String.valueOf(mPosition));
            mDirection = Direction.FORWARD;
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPosition() {
            return mPosition;
        }

        @Override       // override all other functions of Servo that touch the hardware
        public String getConnectionInfo() {
            return mName + " port: unknown";
        }

        public HardwareDevice.Manufacturer getManufacturer() { return Manufacturer.Unknown; }

        public void resetDeviceConfigurationForOpMode() {
            mOpMode.telemetry.addData(mName, "resetDeviceConfigurationForOpMode: ");
        }

        public Servo.Direction getDirection() { return mDirection; }
        public void setDirection(Servo.Direction direction) {
            mDirection = direction;
            mOpMode.telemetry.addData(mName, "direction: " + String.valueOf(mDirection));
        }

        public void scaleRange(double min, double max)
        {
            mScaleMin = min;
            mScaleMax = max;
        }

        public ServoController getController() { return null; }
        public String getDeviceName() { return "AutoLib_TestServo: " + mName; }
        public int getPortNumber()
        {
            return 0;
        }
        public int getVersion() { return 0; }

        public void close() {}

    }

    // a dummy Gyro that just returns default info --
    // useful for testing Gyro code when you don't have real hardware handy
    static public class TestGyro extends TestHardware implements GyroSensor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestGyro(String name, OpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        public void calibrate() {}
        public boolean isCalibrating() { return false; }
        public int getHeading() { return 0; }
        public double getRotationFraction() { return 0; }
        public int rawX() { return 0; }
        public int rawY() { return 0; }
        public int rawZ() { return 0; }
        public void resetZAxisIntegrator() {}
        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // a dummy ColorSensor that just returns default info --
    // useful for testing ColorSensor code when you don't have real hardware handy
    static public class TestColorSensor extends TestHardware implements ColorSensor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this gyro

        public TestColorSensor(String name, OpMode opMode) {
            super();
            mOpMode = opMode;
            mName = name;
        }

        /**
         * Get the Red values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int red() { return 0; }

        /**
         * Get the Green values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int green() { return 0; }

        /**
         * Get the Blue values detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int blue() { return 255; }

        /**
         * Get the amount of light detected by the sensor as an int.
         * @return reading, unscaled.
         */
        public int alpha() { return 0; }

        /**
         * Get the "hue"
         * @return hue
         */
        //@ColorInt
        public int argb() { return 0; }

        /**
         * Enable the LED light
         * @param enable true to enable; false to disable
         */
        public void enableLed(boolean enable) {}

        /**
         * Set the I2C address to a new value.
         *
         */
        public void setI2cAddress(I2cAddr newAddress) {}

        /**
         * Get the current I2C Address of this object.
         * Not necessarily the same as the I2C address of the actual device.
         *
         * Return the current I2C address.
         * @return current I2C address
         */
        public I2cAddr getI2cAddress() {return I2cAddr.zero();}

        public String status() { return "Status okay"; }
        public String getDeviceName() { return "AutoLib_TestGyro: " + mName; }

    }

    // define interface to Factory that creates various kinds of hardware objects
    static public interface HardwareFactory {
        public DcMotor getDcMotor(String name);
        public Servo getServo(String name);
        public GyroSensor getGyro(String name);
        public ColorSensor getColorSensor(String name);
    }

    // this implementation generates test-hardware objects that just log data
    static public class TestHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for logging data

        public TestHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            return new TestMotor(name, mOpMode);
        }

        public Servo getServo(String name){
            return new TestServo(name, mOpMode);
        }

        public GyroSensor getGyro(String name){
            return new TestGyro(name, mOpMode);
        }

        public ColorSensor getColorSensor(String name){
            return new TestColorSensor(name, mOpMode);
        }
    }

    // this implementation gets real hardware objects from the hardwareMap of the given OpMode
    static public class RealHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for access to hardwareMap

        public RealHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            DcMotor motor = null;
            try {
                motor = mOpMode.hardwareMap.dcMotor.get(name);
            }
            catch (Exception e) {
                // okay -- just return null (absent) for this motor
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (motor != null)
                motor.setDirection(DcMotor.Direction.FORWARD);

            return motor;
        }

        public Servo getServo(String name){
            Servo servo = null;
            try {
                servo = mOpMode.hardwareMap.servo.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }

            // just to make sure - a previous OpMode may have set it differently ...
            if (servo != null)
                servo.setDirection(Servo.Direction.FORWARD);

            return servo;
        }

        public GyroSensor getGyro(String name){
            GyroSensor gyro = null;
            try {
                gyro = mOpMode.hardwareMap.gyroSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return gyro;
        }

        public ColorSensor getColorSensor(String name){
            ColorSensor cs = null;
            try {
                cs = mOpMode.hardwareMap.colorSensor.get(name);
            }
            catch (Exception e) {
                // okay - just return null (absent) for this servo
            }
            return cs;
        }

    }

}
