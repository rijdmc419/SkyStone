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

//TODO: Comment this so Daniel can understand!!!

public class SkystoneHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr  = null;
    public DcMotor br = null;
    
    public DcMotor flRevDir = null;
    public DcMotor blRevDir = null;
    public DcMotor frRevDir = null;
    public DcMotor brRevDir = null;
    
    public Servo lfServo = null;
    public Servo rfServo = null;
    public Servo tempServo = null;

    public BNO055IMUHeadingSensor imu = null;

    public ColorSensor lClr = null;
    public DistanceSensor lDist = null;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;


        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

        flRevDir = hwMap.get(DcMotor.class, "fl");
        blRevDir = hwMap.get(DcMotor.class, "bl");
        frRevDir = hwMap.get(DcMotor.class, "fr");
        brRevDir = hwMap.get(DcMotor.class, "br");
        
        //intake2 = hwMap.get(Servo.) //when you type Servo.class for a motor *5head*

        lfServo = hwMap.get(Servo.class, "lfServo");
        rfServo = hwMap.get(Servo.class, "rfServo");
        tempServo = hwMap.get(Servo.class, "tempServo");

        imu = new BNO055IMUHeadingSensor(hwMap.get(BNO055IMU.class, "imu"));
        imu.init(0);

        lClr = hwMap.get(ColorSensor.class, "lClr");
        lDist = hwMap.get(DistanceSensor.class, "lClr");

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flRevDir.setDirection(DcMotor.Direction.REVERSE);
        blRevDir.setDirection(DcMotor.Direction.REVERSE);
        frRevDir.setDirection(DcMotor.Direction.FORWARD);
        brRevDir.setDirection(DcMotor.Direction.FORWARD);


        flRevDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blRevDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frRevDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brRevDir.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brRevDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
