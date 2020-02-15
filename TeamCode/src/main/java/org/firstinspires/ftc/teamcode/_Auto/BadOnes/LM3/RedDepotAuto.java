package org.firstinspires.ftc.teamcode._Auto.BadOnes.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java
@Disabled
@Autonomous(name="LM3 Complex Red Depot")
public class RedDepotAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    ColorSensor clrSnr;
    DistanceSensor dstSnr;
    DcMotor motors[];
    AutoLib.Sequence seq;
    Servo serv;
    float hsv[];

    boolean done;
    public RedDepotAuto(){
        this.msStuckDetectInit = 10000;
    }

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }
    public int lRot(float deg){//neg to counterclockwise, pos to clockwise
        float c = -9.474f;
        float out = c*deg;
        return Math.round(out);
    }
    public int rRot(float deg){ //neg to counterclockwise, pos to clockwise
        float c = 9.648f;
        float out = c*deg;
        return Math.round(out);
    }

    @Override
    public void init(){
        int i = 1;
        float uniPow = 0.33f;
        hsv = new float[3];
        motors = new DcMotor[4];
        robot.init(hardwareMap);

        /*
        //Motors
        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        //Servos
        serv = robot.tempServo;

        //Sensors
        clrSnr = robot.lClr;
        dstSnr = robot.lDist;

        Color.RGBToHSV(clrSnr.red(), clrSnr.green(), clrSnr.blue(), hsv);
        seq = new AutoLib.LinearSequence();

        //Align with the first stone
        seq.add(new AutoLib.ServoStep(serv, 0f));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(36), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));//turns left
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(4), false));

        //Grab the first stone and bring it across
        seq.add(new AutoLib.ServoStep(serv, 1));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-4), false));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-36), false));

        //Deposit it and go back to pick up the second one
        seq.add(new AutoLib.ServoStep(serv, 0));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(40), false));
        seq.add(new AutoLib.ServoStep(serv, 1));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-4), false));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-40), false));

        //Drop off the second one and park on the tape
        seq.add(new AutoLib.ServoStep(serv, 0));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(12), true));
        */
    }
    @Override
    public void start(){
        telemetry.addData("Starting Auto","");
    }
    @Override
    public void loop(){
        if (!done){
            done = seq.loop(); // returns true when we're done
        }
        else{
            telemetry.addData("Sequence finished", "");
        }
    }
    public boolean isStone(float h, float s, float v){ //THIS SHIT BE DEPRECATED
        float stone[] = new float[3];
        stone[0] = 35; //h
        stone[1] = 0.78f; //s
        stone[2] = 0.5f; //v
        boolean out = false;

        if(h >= stone[0]-5 && h <= stone[0]+5){ //r +- standard dev
            out = true;
        }
        else {
            out = false;
        }

        return out;
    }
    @Override
    public void stop(){
        super.stop();
    }
}
