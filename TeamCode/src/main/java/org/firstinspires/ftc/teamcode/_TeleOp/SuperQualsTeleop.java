package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@TeleOp(name="A_Super Mario Quals Teleop")
public class SuperQualsTeleop extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    BNO055IMUHeadingSensor imu;
    DcMotor motors[];
    DcMotor lift, lift2, intake;
    Servo intakeSrv, intakeSrv2;
    Servo lfserv, rfserv;

    @Override
    public void init(){
        motors = new DcMotor[4];
        bot.init(hardwareMap); //DO NOT COMMENT OUT @DANIEL
        imu = bot.imu; //robot gyro TODO why we need this

        //robots drive motors
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        //lift motors
        lift = bot.lift; //lift motor
        lift2 = bot.lift2;

        //intake = bot.intake; //4bar intake motor

        //4bar intake servos
        /**intakeSrv = bot.intakeSrv;
        intakeSrv2 = bot.intakeSrv2;*/

        //foundation servos
        lfserv = bot.lfServo;
        rfserv = bot.rfServo;
    }
    @Override
    public void start(){
        telemetry.addData("Starting Teleop", "");
    }
    @Override
    public void loop(){
        float uniPow; //for 20:1 motors

        //gamepad 1 controls movement
        float tx = gamepad1.right_stick_x; //rotation
        float ty = -gamepad1.left_stick_y;

        float left = (ty + tx/2);
        float right = (ty - tx/2);

        ty = ty*ty*ty;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        float x = gamepad1.left_stick_x; //strafe
        float y = -gamepad1.right_stick_y;

        x=x*x*x;
        y=y*y*y;

        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);

        double theta = Math.atan2(-x, y);
        double heading = theta * 180.0/Math.PI;

        AutoLib.MotorPowers mp = AutoLib.GetSquirrelyWheelMotorPowers(heading);
        double front = mp.Front();
        double back = mp.Back();

        double power = Math.sqrt(x*x + y*y);
        front *= power;
        back *= power;

        if(gamepad1.right_trigger > 0.05f){
            uniPow = 0.5f;
        }
        else {
            uniPow =1f;
        }
        front *= uniPow;
        back *= uniPow;
        left *= uniPow;
        right *= uniPow;

        double fr = Range.clip(back+right, -1, 1);
        double br = Range.clip(front+right, -1, 1);
        double fl = Range.clip(front+left, -1, 1);
        double bl = Range.clip(back+left, -1, 1);


        motors[0].setPower(fr);
        motors[1].setPower(br);
        motors[2].setPower(fl);
        motors[3].setPower(bl);
        telemetry.addData("Heading", imu.getHeading());


        //*gamepad 2 controls mechanisms*
        //lift conrtol (they refers to the drive team) TODO dis work?
        double lftDrv = gamepad2.left_stick_y; //g2 left sitck y controls porwer to lift motors TODO does this need to be negative
        telemetry.addData("Lift Input:", lftDrv);

        lftDrv = lftDrv * lftDrv * lftDrv * lftDrv *lftDrv; //x^5 power so they dont explode it
        telemetry.addData("Power to Lift:", lftDrv);

        // actually make the lift go
        lift.setPower(lftDrv);
        lift2.setPower(lftDrv);


        //4bar intake motor control TODO dis work?
        /**double intkDrv = gamepad2.right_stick_y; //g2 right stick y controls power to 4bar on front of lift TODO Negative?
        telemetry.addData("4Bar Input:", intkDrv);

        intkDrv =intkDrv * intkDrv * intkDrv * intkDrv * intkDrv;
        telemetry.addData("4Bar Power:", intkDrv);

        intake.setPower(intkDrv); //make it thing**/


        //foundation servo control TODO FIXNUMS
        if(gamepad2.a){
            lfserv.setPosition(0f); //up value
            rfserv.setPosition(1f);
        }
        else{
            lfserv.setPosition(1f); //down value (start)
            rfserv.setPosition(0f);
        }
    }
}
