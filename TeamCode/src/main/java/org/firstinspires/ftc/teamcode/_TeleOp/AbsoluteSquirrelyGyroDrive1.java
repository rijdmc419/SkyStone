/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.ToggleButton;
import org.firstinspires.ftc.teamcode._Test._Drive.RobotHardware;

/*
 * TeleOp Mode
 * Enables control of the robot via the gamepad such that the robot moves in the
 * absolute direction and speed indicated by the right joystick, assuming the game console is
 * aligned with the robot when the mode is initiated, and the robot maintains an orientation
 * aligned with the direction of the last left joystick or dpad input.
 *
 * We delegate the work of actually controlling the robot to a Step in Autolib that already does
 * what we want -- a SquirrelyGyroTimedDriveStep drives the robot for a given time along a
 * given direction while keeping the robot pointed at a given heading. See comments in Autolib for
 * more details. In this case, rather than using the Step in an autonomous sequence,
 * our opmode's loop() function updates the power, direction, and heading of the Step from the
 * current gamepad inputs and then runs its loop() function to let it compute the required motor power
 * settings and update the motors, just as it would in an autonomous sequence. We set the time of the
 * Step to a large value (10000) so it won't terminate on its own while we're driving.
 */

@TeleOp(name="AbsoluteSquirrelyGyroDrive1", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class AbsoluteSquirrelyGyroDrive1 extends OpMode {

	AutoLib.SquirrelyGyroTimedDriveStep mStep;
	RobotHardware rh;
	ToggleButton lb;
	ToggleButton rb;

	SensorLib.EncoderGyroPosInt mPosInt;	// position integrator

	/**
	 * Constructor
	 */
	public AbsoluteSquirrelyGyroDrive1() {

	}


	@Override
	public void init() {

		// get hardware
		rh = new RobotHardware();
		rh.init(this);

		// set initial orientation of bot relative to driver (default is 0 degrees == N)
		float initialHeading = 0.0f;	// N
		rh.mIMU.setHeadingOffset(initialHeading);

		// initialize toggle buttons that will count gyro angle correction offset using lb and rb buttons on gamepad
		lb = new ToggleButton(false,360, 0);
		rb = new ToggleButton(false, 360, 0);

		// post instructions to console
		telemetry.addData("AbsoluteSquirrelyGyroDrive1", "");
		telemetry.addData("left stick", " orientation on field");
		telemetry.addData("dpad", " orientation on field");
		telemetry.addData("right stick", " motion on field");
		telemetry.addData("initial heading", initialHeading);

		// construct a PID controller for correcting heading errors
		final float Kp = 0.01f;        // degree heading proportional term correction per degree of deviation
		final float Ki = 0.01f;        // ... integrator term
		final float Kd = 0.0f;         // ... derivative term
		final float KiCutoff = 3.0f;   // maximum angle error for which we update integrator
		SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

		// create a Step that we will use in teleop mode
		mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, rh.mIMU, pid, rh.mMotors, 0, 10000, false);

		// create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
		// use constructor that defaults the wheel type to Normal (not Mecanum or X-Drive)
		int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
		double wheelDiam = 4.0;		    // wheel diameter (in)
		Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);  // example starting position: at origin of field
		SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
				SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
		mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, rh.mIMU, rh.mMotors, countsPerRev, wheelDiam, initialPosn);
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// process gyro correction inputs on lb and rb buttons
		lb.process(gamepad1.left_bumper);
		rb.process(gamepad1.right_bumper);
		rh.mIMU.setHeadingOffset(rb.value()-lb.value());

		// motion direction is on the right stick
		float dx = gamepad1.right_stick_x;
		float dy = -gamepad1.right_stick_y;	// y is reversed :(

		// power is the magnitude of the direction vector
		double power = Math.sqrt(dx*dx + dy*dy);
		mStep.setPower((float) power);

		// make sure we can rotate even if we're not moving
		mStep.setMaxPower((float) 1.0);

		// we don't have a valid direction when inputs are "zero"
		final double MIN_INPUT = 0.1;
		if (power > MIN_INPUT) {
			// direction angle of stick >> the direction we want to move
			double direction = Math.atan2(-dx, dy);    // stick angle: zero = +y, positive CCW, range +-pi
			direction *= 180.0 / Math.PI;        // radians to degrees
			mStep.setDirection((float) direction);
		}

		// vehicle heading (orientation) is on the left stick (near the dpad, which also controls heading)
		float hx = gamepad1.left_stick_x;
		float hy = -gamepad1.left_stick_y;    // y is reversed :(

		double heading = 0;
		boolean setHeading = false;
		double hMag = Math.sqrt(hx*hx + hy*hy);
		if (hMag > MIN_INPUT) {
			// direction angle of stick >> the direction we want to face
			heading = Math.atan2(-hx, hy);    // stick angle: zero = +y, positive CCW, range +-pi
			heading *= 180.0 / Math.PI;        // radians to degrees
			setHeading = true;
		}

		// also allow inputting of orientation on 8-way pad
		if (gamepad1.dpad_up) { heading = 0; setHeading = true; }
		if (gamepad1.dpad_right) { heading = -90; setHeading = true; }
		if (gamepad1.dpad_down) { heading = 180; setHeading = true; }
		if (gamepad1.dpad_left) { heading = 90; setHeading = true; }
		if (gamepad1.dpad_up && gamepad1.dpad_right) { heading = -45; setHeading = true; }
		if (gamepad1.dpad_down && gamepad1.dpad_right) { heading = -135; setHeading = true; }
		if (gamepad1.dpad_down && gamepad1.dpad_left) { heading = 135; setHeading = true; }
		if (gamepad1.dpad_up && gamepad1.dpad_left) { heading = 45; setHeading = true; }

		if (setHeading)
			mStep.setHeading((float) heading);

		// run the control step
		mStep.loop();

		// update position estimate using motor encoders and gyro
		mPosInt.loop();

		// Send telemetry data back to driver station.
		telemetry.addData("position", String.format("%.2f", mPosInt.getX())+", " + String.format("%.2f", mPosInt.getY()));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
	}

}
