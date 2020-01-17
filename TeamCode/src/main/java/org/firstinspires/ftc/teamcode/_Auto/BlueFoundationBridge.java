package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;


@Autonomous(name = "Blue Foundation Complex Bridge Park")
public class BlueFoundationBridge {
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo lfserv, rfserv;
    AutoLib.Sequence seq;
    boolean done = false;

    //@Override
    public void init(){
        motors = new DcMotor[4];
        //robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        lfserv = robot.lfServo;
        rfserv= robot.rfServo;

        float uniPow = 0.33f;

        //TODO: WHY IT NO WORK VV
        //telemetry.addData("Left Servo",lfserv.getPosition());
        //telemetry.addData("Right Servo",rfserv.getPosition());

        seq = new AutoLib.LinearSequence();

        //TODO: Code the thing

        done = false;
    }

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }
    public int lRot(float deg){ //neg to counterclockwise, pos to clockwise
        float c = -9.474f;
        float out = c*deg;
        return  Math.round(out);
    }
    public int rRot(float deg){ //neg to counterclockwise, pos to clockwise
        float c = 9.648f;
        float out = c*deg;
        return Math.round(out);
    }

    //@Override
    public void start(){

    }

    public void loop(){
        if (!done){
            done = seq.loop(); // returns true when we're done
        }
        else{
            //telemetry.addData("Sequence finished", ""); //TODO: Telemetry is borked for some reason
        }
    }
}
