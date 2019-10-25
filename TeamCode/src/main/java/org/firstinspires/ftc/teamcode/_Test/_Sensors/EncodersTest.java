package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;


/**
 * Created by phanau on 10/7/19.
 * Test motor encoders connected to encoder inputs of a RevHub
 */
@Autonomous(name="Test: Encoders Test", group ="Test")
//@Disabled
public class EncodersTest extends OpMode {

    DcMotor mMotors[];
    final String motorNames[] = { "fl", "bl", "fr", "br"};

    public EncodersTest() {
    }

    public void init() {
        // get hardware
        AutoLib.HardwareFactory mf = new AutoLib.RealHardwareFactory(this);
        mMotors = new DcMotor[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            mMotors[i] = mf.getDcMotor(motorNames[i]);
            mMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // FYI: "reversed" motors return counts with opposite sense
        // e.g. mMotors[1].setDirection(DcMotor.Direction.REVERSE);
        // makes left-side motor return positive counts going forward;
    }

    public void loop() {
        // log data to DriverStation
        telemetry.addData("raw counter data -- not reversed", "");
        for (int i = 0; i < mMotors.length; i++)
            telemetry.addData(motorNames[i], mMotors[i].getCurrentPosition());
    }

    public void stop() {}

}
