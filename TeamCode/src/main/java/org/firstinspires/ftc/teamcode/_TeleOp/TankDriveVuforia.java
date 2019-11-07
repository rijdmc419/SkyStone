/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Gonna push this file.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_SkyStone;
import org.firstinspires.ftc.teamcode._Test._Drive.RobotHardware;

/**
 * TeleOp Mode
 *
 * Enables control of the robot via the gamepad
 * Like TankDrivePosInt, uses position integration to estimate where we are on the field
 * plus uses Vuforia data, when available, to update that estimate,
 * and reports that back to the driver station display.
 */

@TeleOp(name="TankDrive Vuforia", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class TankDriveVuforia extends OpMode {

	SensorLib.EncoderGyroPosInt mPosInt;	// position integrator
	VuforiaLib_SkyStone mVLib;
	RobotHardware rh;                       // standard hardware set for these tests

	/**
	 * Constructor
	 */
	public TankDriveVuforia() {
		// override default init timeout to prevent timeouts while starting Vuforia on slow phones.
		// need to do it here so it's in effect BEFORE init() is called.
		this.msStuckDetectInit = 10000;
		this.msStuckDetectStop = 10000;
	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		// get hardware
		rh = new RobotHardware();
		rh.init(this);

		// Start up Vuforia
		mVLib = new VuforiaLib_SkyStone();
		mVLib.init(this);     // pass it this OpMode (so it can do telemetry output)

		// on Ratbot, only two motor encoders are currently hooked up: [1]br, [3]bl
		DcMotor[] encoderMotors = new DcMotor[2];
		encoderMotors[0] = rh.mMotors[1];
		encoderMotors[1] = rh.mMotors[3];

		// create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
		int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
		double wheelDiam = 4.0;		    // wheel diameter (in)
		Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);  // example starting position: at origin of field
		mPosInt = new SensorLib.EncoderGyroPosInt(this, rh.mIMU, encoderMotors, countsPerRev, wheelDiam, initialPosn);
	}

	@Override public void start()
	{
		/** Start tracking the data sets we care about. */
		mVLib.start();
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// tank drive
		// note that if y equal -1 then joystick is pushed all of the way forward.
		float left = -gamepad1.left_stick_y;
		float right = -gamepad1.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		left = Range.clip(left, -1, 1);
		right = Range.clip(right, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		final float scale = 0.3f;
		left =  (float)scaleInput(left) * scale;
		right = (float)scaleInput(right) * scale;

		// write the values to the motors - for tank drive, front and back motors on each side are set the same
		rh.mMotors[0].setPower(right);
		rh.mMotors[1].setPower(right);
		rh.mMotors[2].setPower(left);
		rh.mMotors[3].setPower(left);

		// update position estimate using motor encoders and gyro
		mPosInt.loop();

		// update Vuforia info and, if we have valid location data, update the position integrator with it
		mVLib.loop(true);       // update Vuforia location info

		// don't believe Vuforia data if we're currently turning (blurry image?)
		float angVel = rh.mIMU.getHeadingVelocity();	// in deg/sec
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

		/*
		 * Send telemetry data back to driver station.
		 */
		telemetry.addData("Test", "*** Position Integration ***");
		telemetry.addData("left pwr", String.format("%.2f", left));
		telemetry.addData("right pwr", String.format("%.2f", right));
		telemetry.addData("gamepad1", gamepad1);
		telemetry.addData("gamepad2", gamepad2);
		telemetry.addData("position", String.format("%.2f", mPosInt.getX())+", " + String.format("%.2f", mPosInt.getY()));
		telemetry.addData("rotation rate", angVel);
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
		mVLib.stop();
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
	}

}
