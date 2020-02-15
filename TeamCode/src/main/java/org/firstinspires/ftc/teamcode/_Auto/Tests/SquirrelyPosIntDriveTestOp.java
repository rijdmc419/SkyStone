package org.firstinspires.ftc.teamcode._Auto.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;


/**
 * simple example of using a Step that uses encoder and gyro input to drive to given field positions
 * assuming a "squirrely" drive (Meccanum wheels or X-drive) that can move in any direction independent
 * of the direction the bot is facing.
 * Created by phanau on 5/29/19
 */

// simple example sequence that tests encoder/gyro-based position integration to drive along a given path
@Autonomous(name="Test: Squirrely Pos Int Drive Test")
//@Disabled
public class SquirrelyPosIntDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    SensorLib.EncoderGyroPosInt mPosInt;  // Encoder/gyro-based position integrator to keep track of where we are
    SensorLib.PIDAdjuster mPidAdjuster;     // for interactive adjustment of PID parameters
    SkystoneHardware rh;                       //TODO: Our hardware see if this runs on it
    DcMotor motors[];
    BNO055IMUHeadingSensor imu;

    @Override
    public void init() {

        // get hardware
        rh = new SkystoneHardware();
        rh.init(hardwareMap);

        imu = rh.imu;


        motors = new DcMotor[4];
        motors[0] = rh.fr;
        motors[1] = rh.br;
        motors[2] = rh.fl;
        motors[3] = rh.bl;

        // post instructions to console
        telemetry.addData("PosIntDriveTestOp", "");
        telemetry.addData("requires Meccanum or X-drive", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.01f;        // motor power proportional term correction per degree of deviation
        float Ki = 0f;         // ... integrator term
        float Kd = 0f;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create a PID adjuster for interactive tweaking (see loop() below)
        mPidAdjuster = new SensorLib.PIDAdjuster(this, mPid, gamepad1);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)

        // example starting position: at origin of field
        Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);

        SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
                        SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
                        // SensorLib.EncoderGyroPosInt.DriveType.NORMAL;
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, imu, motors, countsPerRev, wheelDiam, initialPosn);

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.5f;
        float turnPower = 1.0f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
        float tol = 1.0f;   // tolerance in inches

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system in inches
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), 0, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), 0, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), 0, tol, false));
        //mSequence.add(new SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
        //        new Position(DistanceUnit.INCH, -48, -48, 0., 0), 0, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), 0, tol, false));

        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), 90, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), 90, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), 180, tol, false));
        //mSequence.add(new SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
        //        new Position(DistanceUnit.INCH, -48, -48, 0., 0), -90, tol, false));
        mSequence.add(new AutoLib.SqPosIntDriveToStep(this, mPosInt, motors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), 0, tol, false));

        // turn to heading zero to finish up
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, imu, mPid, motors, turnPower, tol, 10));
        mSequence.add(new AutoLib.MoveByTimeStep(motors, 0, 0, true));     // stop all motors

        // start out not-done
        bDone = false;
    }

    public void loop() {
        // report elapsed time to test Suspend/Resume
        telemetry.addData("elapsed time", this.getRuntime());

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
        super.stop();
    }
}

