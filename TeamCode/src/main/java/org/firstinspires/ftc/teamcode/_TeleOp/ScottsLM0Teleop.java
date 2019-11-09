package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@TeleOp(name="top text...bottom text")
public class ScottsLM0Teleop extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];

    @Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.fl;
        motors[2] = robot.bl;
        motors[3] = robot.br;

    }

    @Override
    public void start(){
        telemetry.addData("bruh","Eat Shit");
    }

    @Override
    public void loop(){
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float power = 0.32f;

        left = Range.clip(left, -1, 1); //unnecessary but I'll let it slide
        right = Range.clip(right, -1, 1);
        left = left*left*left;
        right = right*right*right;
        left *= power;
        right *= power;

        motors[0].setPower(left);
        motors[1].setPower(right);
        motors[2].setPower(right);
        motors[3].setPower(left);


    }

    @Override
    public void stop(){

    }
}
