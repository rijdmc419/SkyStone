package org.firstinspires.ftc.teamcode._TeleOp;

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

@TeleOp(name="Scott's Thanksgiving Teleop")
public class GobbleGobbleTeleop extends OpMode{
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo lfserv, rfserv;
    BNO055IMUHeadingSensor imu;
    ColorSensor clrSnr;
    DistanceSensor distSnr;
    Orientation initAngles;

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

        telemetry.addData("IMU Heading", imu.getHeading());
        telemetry.addData("Degrees per turn", imu.getDegreesPerTurn());
        telemetry.addData("Motor Power (%)", uniPow*100);
        telemetry.addData("Is Stone?", isStone(clrSnr.red(), clrSnr.green(), clrSnr.blue()));
        //TODO: make use of this (it's very sporadic
        telemetry.addData("IR Dist",distSnr.getDistance(DistanceUnit.INCH));
        //TODO: Test to see if there's a connection with the alpha value (Cuz these just output zero rn atm)
        telemetry.addData("Red (Light Adjusted))", clrSnr.red()/clrSnr.alpha());
        telemetry.addData("Green (Light Adjusted))", clrSnr.green()/clrSnr.alpha());
        telemetry.addData("Blue (Light Adjusted))", clrSnr.blue()/clrSnr.alpha());
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
    }
    public boolean isStone(int r, int g, int b){ //TODO: Make more accurate (currently only accurate from 3-5in)
        float stone[] = new float[3];
        stone[0] = 68; //r
        stone[1] = 57; //g
        stone[2] = 40; //b
        boolean out = false;

        if(r >= stone[0]-14 && r <= stone[0]+14){ //r +- standard dev
            if(g >= stone[1]-9 && g <= stone[1]+9){ //g +- standard dev
                if(b >= stone[2]-5 && b <= stone[2]+5){ //b +- standard dev
                    out = true;
                }
            }
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
