package org.firstinspires.ftc.teamcode._Libs.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

/**
 * Created by bremm on 9/15/19.
 */

public class SkystoneHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr = null;
    public DcMotor br = null;

    public DcMotor flRevDir = null;
    public DcMotor blRevDir = null;
    public DcMotor frRevDir = null;
    public DcMotor brRevDir = null;

    public DcMotor lift = null;
    public DcMotor lift2 = null;

    //public DcMotor intake = null;

    /**public Servo intakeSrv = null;
    public Servo intakeSrv2 = null;**/
    
    public Servo lfServo = null;
    public Servo rfServo = null;

    public BNO055IMUHeadingSensor imu = null;

    public ColorSensor lClr = null;
    public DistanceSensor lDist = null;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;


        //drive motors
        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

        //lift motors
        lift = hwMap.get(DcMotor.class,  "lift");
        lift2 = hwMap.get(DcMotor.class, "lift2");

        //still dont understand why these are here
        //TODO: LEAVE THESE BE
        flRevDir = hwMap.get(DcMotor.class, "fl");
        blRevDir = hwMap.get(DcMotor.class, "bl");
        frRevDir = hwMap.get(DcMotor.class, "fr");
        brRevDir = hwMap.get(DcMotor.class, "br");

        //4bar intake motor
        //intake = hwMap.get(DcMotor.class, "intake");

        //4bar intake servos
        /**intakeSrv = hwMap.get(Servo.class, "intakeSrv");
        intakeSrv2 = hwMap.get(Servo.class, "intakeSrv2");*/

        //foundation mover servos
        lfServo = hwMap.get(Servo.class, "lfServo");
        rfServo = hwMap.get(Servo.class, "rfServo");

        //gyroscope (i hate it)
        imu = new BNO055IMUHeadingSensor(hwMap.get(BNO055IMU.class, "imu"));
        imu.init(4); //TODO: check if this is the right orientation (lmao no)
        /** notes from the orientation
         * byte AXIS_MAP_CONFIG_BYTE = 0x6;     // Z=-X Y=-Y X=-Z
         * byte AXIS_MAP_SIGN_BYTE = 0x1;       // X Y -Z     ?? if 0x7 -X -Y -Z, X and Y are LH rotation
         */
        imu.setDegreesPerTurn(355.0f);

        //color distance sensor TODO: Not here right now but like soon^tm
        lClr = hwMap.get(ColorSensor.class, "lClr");
        lDist = hwMap.get(DistanceSensor.class, "lClr");

        //gears are difficult
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        //keep dis
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //4bar intake motor setup
        /**intake.setDirection(DcMotor.Direction.FORWARD); //TODO fixnums (direction)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);**/

        //important i think
        //actually not important
        //huh cool
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //why
        flRevDir.setDirection(DcMotor.Direction.REVERSE);
        blRevDir.setDirection(DcMotor.Direction.REVERSE);
        frRevDir.setDirection(DcMotor.Direction.REVERSE);
        brRevDir.setDirection(DcMotor.Direction.REVERSE);

        //paul thinks this good
        flRevDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blRevDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frRevDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brRevDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //still important (why would you not have a motor set to this)
        flRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
