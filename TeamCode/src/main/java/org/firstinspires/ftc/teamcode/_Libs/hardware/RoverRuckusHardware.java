package org.firstinspires.ftc.teamcode._Libs.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by bremm on 10/26/18.
 */

public class RoverRuckusHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr  = null;
    public DcMotor br = null;
    public DcMotor joint = null;
    public DcMotor joint2 = null;
    public DcMotor extend = null;
    public DcMotor extend2 = null;


    public Servo flapServo;
    public CRServo intakeServo;
    public CRServo intakeServo2;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

        joint = hwMap.get(DcMotor.class, "joint");
        joint2 = hwMap.get(DcMotor.class, "joint2");

        extend = hwMap.get(DcMotor.class, "extend");
        extend2 = hwMap.get(DcMotor.class, "extend2");

        flapServo = hwMap.get(Servo.class, "flapServo");
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeServo2 = hwMap.get(CRServo.class, "intakeServo2");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        joint2.setDirection(DcMotor.Direction.REVERSE);
        extend2.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        joint.setDirection(DcMotor.Direction.FORWARD);
        extend.setDirection(DcMotor.Direction.FORWARD);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        joint.setPower(0);
        joint2.setPower(0);
        extend.setPower(0);
        extend2.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flapServo.setPosition(0);
        intakeServo.setPower(0);
        intakeServo2.setPower(0);
    }
}
