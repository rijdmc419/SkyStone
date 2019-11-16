package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@TeleOp(name="LM1 SERVO Teleop")
public class LM1TeleopSERVO extends OpMode{
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo greg;
    BNO055IMU gyr0;
    boolean A=false;
    boolean whichA=false;

    @Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap);
        //TODO: see if we can switch the br and fr here rather than in the config file (we'll test that on the Fri before LM1)
        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        greg = robot.greg;

        gyr0 = robot.gyr0;
        BNO055IMU.Parameters gParams = new BNO055IMU.Parameters();
        gParams.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
    }

    @Override
    public void start(){
        telemetry.addData("Starting Teleop", "");
    }

    @Override
    public void loop(){
        float uniPow = 0.33f; //for 20:1 motors
        float tx = gamepad1.left_stick_x; //rotation
        float ty = -gamepad1.left_stick_y;	//forward & back -- y is reversed :(
        float left = (ty + tx/2);
        float right = (ty - tx/2);

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        float x = gamepad1.right_stick_x; //strafe
        float y = -gamepad1.right_stick_y;//forward & back

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

        //TODO: Test wether cubing the powers when we take them as inputs as oposed to here works.
        //front = front*front*front;
        //back = back*back*back;
        //left = left*left*left;
        //right = right*right*right;

        //TODO: Also test these
        front *= uniPow;
        back *= uniPow;
        left *= uniPow;
        right *= uniPow;

        double fr = Range.clip(front+right, -1, 1);
        double br = Range.clip(back+right, -1, 1);
        double fl = Range.clip(front+left, -1, 1);
        double bl = Range.clip(back+left, -1, 1);

        motors[0].setPower(fr);
        motors[1].setPower(br);
        motors[2].setPower(fl);
        motors[3].setPower(bl);
        telemetry.addData("", gamepad1);
        telemetry.addData("Tempature: ", gyr0.getTemperature());

        if(gamepad2.a){
            A = true;
        }
        else if(A=true){
            A = false;
            whichA= !whichA;
            if(whichA){
                greg.setPosition(0f); //down value
            }
            else{
                greg.setPosition(-1f); //up value start
            }
        }
    }

    @Override
    public void stop(){

    }
}
