package org.firstinspires.ftc.teamcode._Libs.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

/**
 * Created by bremm on 9/15/19.
 */

public class SkystoneHardware {
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor fr  = null;
    public DcMotor br = null;

    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;


        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

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
    }
}
