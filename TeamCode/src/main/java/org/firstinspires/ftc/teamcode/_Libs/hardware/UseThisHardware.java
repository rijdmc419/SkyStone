package org.firstinspires.ftc.teamcode._Libs.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: Add all the other shit from the whiteboard image (in Daniel-Scott texts)
public class UseThisHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr  = null;
    public DcMotor br = null;

    public DcMotor lift = null;
    public DcMotor lift2 = null;

    public DcMotor top = null;

    public Servo lFound = null;
    public Servo rFound = null;

    public Servo intake = null;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        lift = hwMap.get(DcMotor.class, "lift");
        lift2 = hwMap.get(DcMotor.class, "lift2");
        top = hwMap.get(DcMotor.class, "top");

        lFound = hwMap.get(Servo.class, "lFound");
        rFound = hwMap.get(Servo.class, "rFound");
        intake = hwMap.get(Servo.class, "intake");

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);//+ === up,  - === down
        lift2.setDirection(DcMotor.Direction.FORWARD);
        top.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
