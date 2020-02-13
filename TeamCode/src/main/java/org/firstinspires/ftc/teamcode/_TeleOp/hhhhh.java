package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.UseThisHardware;

@TeleOp(name="TTTTTTTest")
public class hhhhh extends OpMode {
    //Hardware shiz my niz
    UseThisHardware bot = new UseThisHardware();
    DcMotor mMotors[];
    DcMotor rLift, lLift;
    DcMotor top;

    @Override
    public void init(){
        mMotors = new DcMotor[4];
        bot.init(hardwareMap);

        mMotors[0] = bot.fr;
        mMotors[1] = bot.br;
        mMotors[2] = bot.fl;
        mMotors[3] = bot.bl;

        rLift = bot.lift;
        lLift = bot.lift2;
        top = bot.top;
    }

    @Override
    public void start(){
        telemetry.addData("Starting Teleop", "");
    }

    @Override
    public void loop(){
        float liftPow = 0.17f;

        double x = gamepad1.left_stick_x;
        double y = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_y, -1, 1);

        double theta = (Math.atan2(y, x) - Math.toRadians(45)) % (2 * Math.PI);//rotate the input

        if (theta < 0) {
            theta = 2 * Math.PI + theta;
        }

        double mag = Math.pow(Math.sqrt(x*x + y*y), 2);//square input magnitude

        if (3 * Math.PI / 4 >= theta && theta >= Math.PI / 4) {//normalize input from unit circle to unit square
            y = mag;
            x = y / Math.tan(theta);
        } else if ((Math.PI / 4 >= theta && theta >= 0) || (7 * Math.PI / 4 <= theta && theta <= 2 * Math.PI)) {
            x = mag;
            y = x * Math.tan(theta);
        } else if (5 * Math.PI / 4 <= theta && theta <= 7 * Math.PI / 4) {
            y = -mag;
            x = y / Math.tan(theta);
        } else if (3 * Math.PI / 4 <= theta && theta <= 5 * Math.PI / 4) {
            x = -mag;
            y = x * Math.tan(theta);
        }

        double rotation = Math.pow(gamepad1.right_stick_x, 1) * Math.abs(gamepad1.right_stick_x);


        double[] motorPowers = new double[]{x + rotation, y + rotation, x - rotation, y - rotation};//combine translation and rotation
        if (Math.abs(motorPowers[0]) > 1 || Math.abs(motorPowers[1]) > 1 || Math.abs(motorPowers[2]) > 1 || Math.abs(motorPowers[3]) > 1) {//if a power is greater than 1 or less than -1, normalize all the motor powers, keeping the proportion the same
            double maxPower = GetMaxAbsMotorPower();
            motorPowers = new double[]{adjustPower(motorPowers[0] / maxPower), adjustPower(motorPowers[1] / maxPower), adjustPower(motorPowers[2] / maxPower), adjustPower(motorPowers[3] / maxPower)};
        } else {
            motorPowers = new double[]{adjustPower(motorPowers[0]), adjustPower(motorPowers[1]), adjustPower(motorPowers[2]), adjustPower(motorPowers[3])};
        }


        mMotors[0].setPower(motorPowers[0]);
        mMotors[1].setPower(motorPowers[1]);
        mMotors[2].setPower(motorPowers[2]);
        mMotors[3].setPower(motorPowers[3]);

        double liftInput = gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y * liftPow;
        liftInput *= -1;
        double liftF = Range.clip(liftInput, -1, 1);

        telemetry.addData("Boof", liftF);
        top.setPower(liftF);
    }
    @Override
    public void stop(){
        super.stop();
    }
}
