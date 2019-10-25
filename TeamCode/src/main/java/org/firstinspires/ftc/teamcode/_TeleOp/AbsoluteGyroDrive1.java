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

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Test._Drive.RobotHardware;

/*
 * TeleOp Mode
 * Enables control of the robot via the gamepad such that the robot moves in the
 * absolute direction and speed indicated by the left joystick, assuming the game console is
 * aligned with the robot when the mode is initiated. This code assumes a normal drive type,
 * so the robot always points in the direction it's going and it always moves forward.
 *
 * We delegate the work of actually controlling the robot to a Step in Autolib that already does
 * what we want -- an AzimuthTimedDriveStep drives the robot for a given time along a
 * given direction. See comments in Autolib for more details.
 * In this case, rather than using the Step in an autonomous sequence,
 * our opmode's loop() function updates the power and direction of the Step from the
 * current gamepad inputs and then runs its loop() function to let it compute the required motor power
 * settings and update the motors, just as it would in an autonomous sequence. We set the time of the
 * Step to a large value (10000) so it won't terminate on its own while we're driving.
 */

@TeleOp(name="AbsoluteGyroDrive1", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class  AbsoluteGyroDrive1 extends OpMode {

	AutoLib.AzimuthTimedDriveStep mStep;
	RobotHardware rh;

	/**
	 * Constructor
	 */
	public AbsoluteGyroDrive1() {

	}

	@Override
	public void init() {
		// get hardware
		rh = new RobotHardware();
		rh.init(this);

		// create a Step that we will use in teleop mode
		mStep = new AutoLib.AzimuthTimedDriveStep(this, 0, rh.mIMU, null, rh.mMotors, 0, 0.25f,10000, false);
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// motion direction and power is on the left stick
		float dx = gamepad1.left_stick_x;
		float dy = -gamepad1.left_stick_y;	// y is reversed :(

		// power is the magnitude of the direction vector
		double power = Math.sqrt(dx*dx + dy*dy);

		// don't update direction when it's essentially undefined (zero inputs)
		final double MIN_INPUT = 0.1;
		if (power > MIN_INPUT) {
			// direction angle of stick >> the direction we want to move
			double direction = Math.atan2(-dx, dy);    // stick angle: zero = +y, positive CCW, range +-pi
			direction *= 180.0 / Math.PI;        // radians to degrees

			// update the control step we're using to control the motors and then run it
			mStep.setDirection((float) direction);
			mStep.setPower((float) power);
		}
		else
			mStep.setPower(0);
		mStep.loop();
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
