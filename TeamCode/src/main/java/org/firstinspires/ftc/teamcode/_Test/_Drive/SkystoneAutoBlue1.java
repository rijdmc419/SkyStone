package org.firstinspires.ftc.teamcode._Test._Drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_SkyStone;

import java.util.ArrayList;


/**
 * simple example of using a Step that uses encoder and gyro input to drive to given field positions
 * assuming a "squirrely" drive (Meccanum wheels or X-drive) that can move in any direction independent
 * of the direction the bot is facing.
 * Created by phanau on 5/29/19
 */

// simple example sequence that tests encoder/gyro-based position integration to drive along a given path
@Autonomous(name="Test: Skystone Auto Blue 1", group ="Test")
//@Disabled
public class SkystoneAutoBlue1 extends OpMode {

    // return done when we're within tolerance distance of target position
    class PositionTerminatorStep extends AutoLib.MotorGuideStep {

        OpMode mOpMode;
        SensorLib.PositionIntegrator mPosInt;
        Position mTarget;
        double mTol;
        double mPrevDist;

        public PositionTerminatorStep(OpMode opmode, SensorLib.PositionIntegrator posInt, Position target, double tol) {
            mOpMode = opmode;
            mPosInt = posInt;
            mTarget = target;
            mTol = tol;
            mPrevDist = 1e6;    // infinity
        }

        @Override
        public boolean loop() {
            super.loop();
            Position current = mPosInt.getPosition();
            double dist = Math.sqrt((mTarget.x-current.x)*(mTarget.x-current.x) + (mTarget.y-current.y)*(mTarget.y-current.y));
            if (mOpMode != null) {
                mOpMode.telemetry.addData("PTS target", String.format("%.2f", mTarget.x) + ", " + String.format("%.2f", mTarget.y));
                mOpMode.telemetry.addData("PTS current", String.format("%.2f", current.x) + ", " + String.format("%.2f", current.y));
                mOpMode.telemetry.addData("PTS dist", String.format("%.2f", dist));
            }
            boolean bDone = (dist < mTol);

            // try to deal with "circling the drain" problem -- when we're close to the tolerance
            // circle, but we can't turn sharply enough to get into it, we circle endlessly --
            // if we detect that situation, just give up and move on.
            // simple test: if we're close but the distance to the target increases, we've missed it.
            if (dist < mTol*4 && dist > mPrevDist)
                bDone = true;
            mPrevDist = dist;

            return bDone;
        }
    }

    // guide step that uses a gyro and a position integrator to determine how to guide the robot to the target
    class SqGyroPosIntGuideStep extends AutoLib.SquirrelyGyroGuideStep {

        OpMode mOpMode;
        Position mTarget;
        SensorLib.EncoderGyroPosInt mPosInt;
        double mTol;
        float mMaxPower;
        float mMinPower = 0.25f;
        float mSgnPower;

        public SqGyroPosIntGuideStep(OpMode opmode, SensorLib.EncoderGyroPosInt posInt, Position target, float heading,
                                     SensorLib.PID pid, ArrayList<AutoLib.SetPower> motorsteps, float power, double tol) {
            // this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));
            super(opmode, 0, heading, posInt.getGyro(), pid, motorsteps, power);
            mTarget = target;
            mPosInt = posInt;
            mTol = tol;
            mMaxPower = Math.abs(power);
            mSgnPower = (power > 0) ? 1 : -1;
        }

        public boolean loop() {
            // run the EncoderGyroPosInt to update its position based on encoders and gyro
            mPosInt.loop();

            // update Vuforia info and, if we have valid location data, update the position integrator with it
            mVLib.loop(true);       // update Vuforia location info

            // don't believe Vuforia data if we're currently turning (blurry image?)
            float angVel = mPosInt.getGyro().getHeadingVelocity();	// in deg/sec
            boolean turningTooFast = Math.abs(angVel) > 10.0;

            // if we have Vuforia location data, update the position integrator from it.
            // use STATUS and STATUS_INFO associated with the sample to decide how much to believe it.
            if (mVLib.haveLocation() && mVLib.getTrackableStatusInfo() == TrackableResult.STATUS_INFO.NORMAL && !turningTooFast) {
                if (mVLib.getTrackableStatus() == TrackableResult.STATUS.TRACKED)
                    mPosInt.setPosition(mVLib.getFieldPosition());
                else
                if (mVLib.getTrackableStatus() == TrackableResult.STATUS.EXTENDED_TRACKED)
                    mPosInt.setPosition(mVLib.getFieldPosition(), 0.5f);
            }

            // update the SquirrelyGyroGuideStep direction to continue heading for the target
            // while maintaining the heading (orientation) given for this Step
            float direction = (float) HeadingToTarget(mTarget, mPosInt.getPosition());
            super.setDirection(direction);

            // when we're close to the target, reduce speed so we don't overshoot
            Position current = mPosInt.getPosition();
            float dist = (float)Math.sqrt((mTarget.x-current.x)*(mTarget.x-current.x) + (mTarget.y-current.y)*(mTarget.y-current.y));
            float brakeDist = (float)mTol * 5;  // empirical ...
            if (dist < brakeDist) {
                float power = mSgnPower * (mMinPower + (mMaxPower-mMinPower)*(dist/brakeDist));
                super.setMaxPower(power);
            }

            // run the underlying GyroGuideStep and return what it returns for "done" -
            // currently, it leaves it up to the terminating step to end the Step
            return super.loop();
        }

        private double HeadingToTarget(Position target, Position current) {
            double headingXrad = Math.atan2((target.y - current.y), (target.x - current.x));        // pos CCW from X-axis
            double headingYdeg = SensorLib.Utils.wrapAngle(Math.toDegrees(headingXrad) - 90.0);     // pos CCW from Y-axis
            if (mOpMode != null) {
                mOpMode.telemetry.addData("GPIGS.HTT target", String.format("%.2f", target.x) + ", " + String.format("%.2f", target.y));
                mOpMode.telemetry.addData("GPIGS.HTT current", String.format("%.2f", current.x) + ", " + String.format("%.2f", current.y));
                mOpMode.telemetry.addData("GPIGS.HTT heading", String.format("%.2f", headingYdeg));
            }
            return headingYdeg;
        }
    }

    // Step: drive to given absolute field position while facing in the given direction using given EncoderGyroPosInt
    class SqPosIntDriveToStep extends AutoLib.GuidedTerminatedDriveStep {

        SensorLib.EncoderGyroPosInt mPosInt;
        Position mTarget;
        AutoLib.GyroGuideStep mGuideStep;
        PositionTerminatorStep mTerminatorStep;

        public SqPosIntDriveToStep(OpMode opmode, SensorLib.EncoderGyroPosInt posInt, DcMotor[] motors,
                                   float power, SensorLib.PID pid, Position target, float heading, double tolerance, boolean stop)
        {
            super(opmode, new SqGyroPosIntGuideStep(opmode, posInt, target, heading, pid, null, power, tolerance),
                    new PositionTerminatorStep(opmode, posInt, target, tolerance),
                    motors);

            mPosInt = posInt;
            mTarget = target;
        }

    }



    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    SensorLib.EncoderGyroPosInt mPosInt;    // Encoder/gyro-based position integrator to keep track of where we are
    SensorLib.PIDAdjuster mPidAdjuster;     // for interactive adjustment of PID parameters
    RobotHardware rh;                       // standard hardware set for these tests
    VuforiaLib_SkyStone mVLib;

    /**
     * Constructor
     */
    public SkystoneAutoBlue1() {
        // override default init timeout to prevent timeouts while starting Vuforia on slow phones.
        // need to do it here so it's in effect BEFORE init() is called.
        this.msStuckDetectInit = 10000;
        this.msStuckDetectStop = 10000;
    }

    @Override
    public void init() {

        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // post instructions to console
        telemetry.addData("Skystone auto blue 1", "");
        telemetry.addData("requires Meccanum or X-drive", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders with ");
        telemetry.addData("", "occasional Vuforia updates ");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.01f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.01f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create a PID adjuster for interactive tweaking (see loop() below)
        mPidAdjuster = new SensorLib.PIDAdjuster(this, mPid, gamepad1);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*40;		// for 40:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)

        // initial position and orientation of bot is along Blue wall near the red loading zone facing the Red side
        rh.mIMU.setHeadingOffset(180);  // initially bot is facing in (Vuforia) field -Y direction, where (for us) +Y is bearing zero
        Position initialPosn = new Position(DistanceUnit.INCH, -36.0, 63.0, 0.0, 0);
        SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
                        SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, rh.mIMU, rh.mMotors, countsPerRev, wheelDiam, initialPosn);

        // Start up Vuforia
        mVLib = new VuforiaLib_SkyStone();
        mVLib.init(this);     // pass it this OpMode (so it can do telemetry output)

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.4f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        float tol = 1.0f;   // tolerance in inches

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side
        // Vuforia convention is bearing zero = +X while our code uses bearing zero = +Y, so there's an offset
        final float boff = -90;  // bearing offset of our convention (+Y to rear of field) vs. Vuforia's (+Y to Blue side)
        // coordinates and bearings below are in Vuforia terms to be compatible with Vuforia position updates if we use them.

        // fetch a SkyStone (for now we assume we know where it is) and pull it out of line
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -44, 32, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -44, 40, 0., 0), -90+boff, tol, false));

        // go to the Blue Skybridge
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90+boff, tol, false));

        // go to the Blue Foundation and pull it into the Blue Building Area
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 32, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 62, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 62, 0., 0), -90+boff, tol, false));

        // slide out of the corridor left by positioning the Foundation
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90+boff, tol, true));

        // return to the quarry for a second SkyStone
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 32, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 40, 0., 0), -90+boff, tol, false));

        // bring it to the audience end of the Foundation via the Blue Skybridge
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 24, 60, 0., 0), 0+boff, tol, false));

        // park under the SkyBridge
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), 0+boff, tol, false));

        // stop all motors
        mSequence.add(new AutoLib.MoveByTimeStep(rh.mMotors, 0, 0, true));

        // start out not-done
        bDone = false;
    }

    @Override public void start()
    {
        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    public void loop() {

        // optionally adjust PID for current robot configuration
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
        mVLib.stop();
    }
}

