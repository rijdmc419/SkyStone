package org.firstinspires.ftc.teamcode._TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@TeleOp(name="a_LM3 Teleop")
public class GobbleGobbleTeleop extends OpMode{
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo lfserv, rfserv;
    Servo tserv;
    BNO055IMUHeadingSensor imu;
    ColorSensor clrSnr;
    DistanceSensor distSnr;
    Orientation initAngles;
    float[] hsv = new float[3];

    @Override
    public void init(){

        motors = new DcMotor[4];
        robot.init(hardwareMap);
        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        lfserv = robot.lfServo;
        rfserv = robot.rfServo;
        tserv = robot.tempServo;
        clrSnr = robot.lClr;
        distSnr = robot.lDist;

        imu = robot.imu; //TODO: Setup gyro based strafing
    }

    @Override
    public void start(){
        telemetry.addData("Starting Teleop", "");
    }

    @Override
    public void loop(){
        float uniPow; //for 20:1 motors
        float tx = gamepad1.right_stick_x; //rotation
        float ty = -gamepad1.left_stick_y;	//forward & back -- y is reversed :(


        ty = ty*ty*ty;

        float left = (ty + tx/2);
        float right = (ty - tx/2);

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

        if(gamepad1.right_trigger > 0){ //slow mode
            uniPow = 1f;
        }
        else {
            uniPow =0.5f;
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

        Color.RGBToHSV(clrSnr.red(), clrSnr.green(), clrSnr.blue(), hsv);
        telemetry.addData("IMU Heading", imu.getHeading());
        telemetry.addData("Motor Power (%)", Math.round(uniPow*100));
        telemetry.addData("Is Stone?", isStone(Math.round(hsv[0]), Math.round(hsv[1]), Math.round(hsv[2])));
        //TODO: make use of this (it's very sporadic)
        telemetry.addData("IR Dist",distSnr.getDistance(DistanceUnit.INCH));
        //TODO: Test to see if there's a connection with the alpha value (Cuz these just output zero rn atm)
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);

        telemetry.addData("Alpha", clrSnr.alpha());
        telemetry.addData("Red", clrSnr.red());
        telemetry.addData("Green", clrSnr.green());
        telemetry.addData("Blue", clrSnr.blue());

        telemetry.addData("", gamepad1);
        telemetry.update();


        if(gamepad2.a){ //Foundation servo flaps
            lfserv.setPosition(0f); //down value
            rfserv.setPosition(1f);
        }
        else{
            lfserv.setPosition(1f); //up value (start)
            rfserv.setPosition(0f);
        }

        if(gamepad2.b){
            tserv.setPosition(1f); //down val
        }
        else{
            tserv.setPosition(0f); //up val (default)
        }
    }
    public boolean isStone(float h, float s, float v){ //TODO: Make more accurate (currently only accurate from 3-5in)
        float stone[] = new float[3];
        stone[0] = 35; //h
        stone[1] = 0.78f; //s
        stone[2] = 0.5f; //v
        boolean out = false;

        if(h >= stone[0]-5 && h <= stone[0]+5 && distSnr.getDistance(DistanceUnit.INCH) <= 10){ //r +- standard dev
            out = true;
        }
        else {
            out = false;
        }

        return out;
    }
    @Override
    public void stop(){

    }
}
