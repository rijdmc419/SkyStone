package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;
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
        int flLiftLim = -145, frLiftLim = -155, blLiftLim = 10, brliftlim = 0; //lift == lift2 + 10
        boolean atoggle = false;                                                //left == lift2
        double liftInput;                                                      //right == lift
        float uniPow; //for 20:1 motors
        float liftPow = 0.75f;
        float topPow = 0.33f;
        float tx = gamepad1.right_stick_x; //rotation
        float ty = -gamepad1.left_stick_y;	//forward & back -- y is reversed :(

        float left = (ty + tx/2);
        float right = (ty - tx/2);

        ty = ty*ty*ty;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        float x = gamepad1.left_stick_x; //strafe
        float y = -gamepad1.right_stick_y;//forward & back

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


        mMotors[0].setPower(fr);
        mMotors[1].setPower(br);
        mMotors[2].setPower(fl);
        mMotors[3].setPower(bl);

        //Lift styuf
        telemetry.addData("leftMotor", lLift.getCurrentPosition());
        telemetry.addData("rightMotor", rLift.getCurrentPosition());

        float liftOutput = Range.clip( gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y * liftPow,-1 ,1);
        telemetry.addData("Lift Output", liftOutput);

        rLift.setPower(liftOutput);
        lLift.setPower(liftOutput);
        /*
        fl = 23
        fr = 21

        bl = 170
        br = 171
        */

        //4-bar linkage styuf
        float topOutput = Range.clip(gamepad2.right_stick_y * gamepad2.right_stick_y * gamepad2.right_stick_y * topPow, -1, 1);
        top.setPower(topOutput);
    }
    @Override
    public void stop(){
        super.stop();
    }
}
