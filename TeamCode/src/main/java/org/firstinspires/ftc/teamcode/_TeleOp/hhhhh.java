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

    }

    @Override
    public void start(){
        telemetry.addData("Starting Teleop", "");
        rLift.setTargetPosition(-40); //lift == right side
        lLift.setTargetPosition(-50);
    }

    @Override
    public void loop(){
        int flLiftLim = -145, frLiftLim = -155, blLiftLim = 10, brliftlim = 0; //lift == lift2 + 10
        boolean apress = false;                                                //left == lift2
        double liftInput;                                                      //right == lift
        float uniPow; //for 20:1 motors
        float liftPow = 0.67f;
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

        if(gamepad2.a){
            rLift.setTargetPosition(frLiftLim); //lift == right side
            lLift.setTargetPosition(flLiftLim); //lift2 == left side
        }
        else{
            rLift.setTargetPosition(brliftlim); //lift == right side
            lLift.setTargetPosition(blLiftLim); //lift2 == left side
        }
        /*
        liftInput = gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y * liftPow;
        double liftF = Range.clip(liftInput, -1, 1);

        telemetry.addData("Boof", liftF); */
        rLift.setPower(liftPow);
       lLift.setPower(liftPow);
    }
    @Override
    public void stop(){
        super.stop();
    }
}
