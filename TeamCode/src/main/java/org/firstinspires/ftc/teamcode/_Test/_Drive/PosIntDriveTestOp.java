package org.firstinspires.ftc.teamcode._Test._Drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

import java.util.ArrayList;


/**
 * simple example of using a Step that uses encoder and gyro input to drive to given field positions.
 * Created by phanau on 12/15/18
 */

// simple example sequence that tests encoder/gyro-based position integration to drive along a given path
@Autonomous(name="Test: Pos Int Drive Test", group ="Test")
//@Disabled
public class PosIntDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    SensorLib.EncoderGyroPosInt mPosInt;    // Encoder/gyro-based position integrator to keep track of where we are
    SensorLib.PIDAdjuster mPidAdjuster;     // for interactive adjustment of PID parameters
    RobotHardware rh;                       // standard hardware set for these tests

    @Override
    public void init() {
        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // post instructions to console
        telemetry.addData("PosIntDriveTestOp", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create a PID adjuster for interactive tweaking (see loop() below)
        mPidAdjuster = new SensorLib.PIDAdjuster(this, mPid, gamepad1);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0); // example starting position: at origin of field
        mPosInt = new SensorLib.EncoderGyroPosInt(this, rh.mIMU, rh.mMotors, countsPerRev, wheelDiam, initialPosn);


        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.25f;
        float turnPower = 0.25f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of movements to the sequence
        float tol = 1.0f;   // tolerance in inches
        float timeout = 2.0f;   // seconds

        // these position integrator steps use the encoder-based position integrator and IMU-gyro to move
        // the robot to a sequence of positions specified in absolute field coordinate system in inches
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, true));

        mSequence.add((new AutoLib.LogTimeStep(this, "stopped!", 5)));

        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, -movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, -movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        mSequence.add(new AutoLib.PosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, true));

        mSequence.add((new AutoLib.LogTimeStep(this, "stopped!", 5)));

        // turn to heading zero to finish up
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, rh.mIMU, mPid, rh.mMotors, turnPower, tol, 10));
        mSequence.add(new AutoLib.MoveByTimeStep(rh.mMotors, 0, 0, true));     // stop all motors

        // start out not-done
        bDone = false;
    }

    public void loop() {

        if (gamepad1.y)
            bSetup = true;      // Y button: enter "setup mode" using controller inputs to set Kp and Ki
        if (gamepad1.x)
            bSetup = false;     // X button: exit "setup mode"
        if (bSetup) {           // "setup mode"
            mPidAdjuster.loop();
            return;
        }

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop() {
    }
}

