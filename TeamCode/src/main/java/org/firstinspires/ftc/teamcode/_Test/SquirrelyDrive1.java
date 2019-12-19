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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Test._Drive.RobotHardware;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad using Squirrely Wheels
 */
@TeleOp(name="SquirrelyDrive1", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class SquirrelyDrive1 extends OpMode {

	RobotHardware rh;

	/**
	 * Constructor
	 */
	public SquirrelyDrive1() { }

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
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		final double deadband = 0.05;

		// turning drive is on the left stick
		float tx = gamepad1.left_stick_x;
		float ty = -gamepad1.left_stick_y;	// y is reversed :(

		// deadband to avoid zero-stick drifting
		if (Math.abs(tx) < deadband) tx = 0;
		if (Math.abs(ty) < deadband) ty = 0;

		// scale the drive joystick value to make it easier to control the robot more precisely at slower speeds.
		// don't scale the turn value - doing so makes turning much to sluggish.
		ty = (float)scaleInput(ty);

		// compute the drive+turn powers
		float left = (ty + tx/2);
		float right = (ty - tx/2);

		// squirrely drive is on the right stick
		float x = gamepad1.right_stick_x;
		float y = -gamepad1.right_stick_y;	// y is reversed :(

		// clip the values so that the values never exceed +/- 1
		x = Range.clip(x, -1, 1);
		y = Range.clip(y, -1, 1);

		// direction angle of stick >> the relative direction we want to move
		double theta = Math.atan2(-x, y);	// stick angle: zero = +y, positive CCW, range +-pi
		double heading = theta * 180.0/Math.PI;		// radians to degrees

		// compute left-side front and back wheel relative speeds needed to go in desired direction
		AutoLib.MotorPowers mp = AutoLib.GetSquirrelyWheelMotorPowers(heading);
		double rightFacing = mp.RightFacing();
		double leftFacing = mp.LeftFacing();

		// power is the magnitude of the stick vector
		double power = Math.sqrt(x*x + y*y);

		// deadband to avoid zero-stick drifting
		if (Math.abs(power) < deadband)
			power = 0;

		// scale the joystick values to make it easier to control the robot more precisely at slower speeds.
		power =  scaleInput(power);

		// scale the values by the desired power
		rightFacing *= power;
		leftFacing *= power;

		// combine turning and squirrely drive inputs assuming "standard" arrangement
		// of mecanum wheels with roller axles pointing toward bot center
		// which is equivalent to X-drive.
		double fr = Range.clip(leftFacing+right, -1, 1);
		double br = Range.clip(rightFacing+right, -1, 1);
		double fl = Range.clip(rightFacing+left, -1, 1);
		double bl = Range.clip(leftFacing+left, -1, 1);

		// write the values to the motors
		rh.mMotors[0].setPower(fr);
		rh.mMotors[1].setPower(br);
		rh.mMotors[2].setPower(fl);
		rh.mMotors[3].setPower(bl);

		/*
		 * Send telemetry data back to driver station.
		 */
		telemetry.addData("SquirrelyDrive1", "*** v1.5 ***");
		telemetry.addData("front left/right power:", "%.2f %.2f", fl, fr);
		telemetry.addData("back left/right power:", "%.2f %.2f", bl, br);
		telemetry.addData("heading:", "%.2f", heading);
		telemetry.addData("gamepad1:", gamepad1);
		//telemetry.addData("gamepad2", gamepad2);
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

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
