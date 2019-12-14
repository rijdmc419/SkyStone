package org.firstinspires.ftc.teamcode._Test._Drive;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;


/**
 * simple example of using a Step that makes a bot with "squirrely wheels" drive along a given course
 * Created by phanau on 10/31/16.
 */


// simple example sequence that tests time based "squirrely wheel" drive steps to drive along a prescribed path
// while stabilizing robot orientation with gyro inputs
@Autonomous(name="Test: Squirrely Gyro Drive Test 1", group ="Test")
//@Disabled
public class SquirrelyGyroDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    RobotHardware rh;                       // standard hardware set for these tests

    @Override
    public void init() {
        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float power = 0.3f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CCW from initial heading
        float leg = 3.0f;  // time along each leg of the polygon
        SensorLib.PID pid = null;         // use default PID provided by the step

        // drive a square while maintaining constant orientation (0)
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, -90, 0, rh.mIMU, pid, rh.mMotors, power, leg/2, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this,   0, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this,  90, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 180, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 270, 0, rh.mIMU, pid, rh.mMotors, power, leg/2, false));

        // ... and then a diamond
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, -45, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this,  45, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 135, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 225, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));

        // ... and then sort of a polygonal circle
        int n = 60;     // number of sides
        for (int i=0; i<=n; i++) {
            float heading = 360*i/n - 90;
            boolean stop = (i==n);
            int t = (i==0 || i==n) ? 2 : 4;
            mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, heading, 0, rh.mIMU, pid, rh.mMotors, power, t*leg/n, stop));
        }

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
    }
}

