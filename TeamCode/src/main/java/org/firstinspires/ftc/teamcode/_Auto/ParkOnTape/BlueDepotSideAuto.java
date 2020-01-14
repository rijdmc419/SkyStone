package org.firstinspires.ftc.teamcode._Auto.ParkOnTape;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;
//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java

@Autonomous(name="BLUE Depot Park Close")
public class BlueDepotSideAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo lfserv, rfserv;
    AutoLib.Sequence seq;
    AutoLib.Sequence foundSeq;
    boolean done;


    @Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        lfserv = robot.lfServo;
        rfserv= robot.rfServo;

        float uniPow = 0.33f;

        telemetry.addData("Left Mover", lfserv);
        telemetry.addData("Right Mover", rfserv);

        // foundSeq = new AutoLib.ConcurrentSequence();
        seq = new AutoLib.LinearSequence();

        // foundSeq.add(new AutoLib.ServoStep(lfserv, 0));
        //foundSeq.add(new AutoLib.ServoStep(rfserv, 0));
        //start sequence
     //   seq.add(new AutoLib.ServoStep(lfserv, 0));
      //  seq.add(new AutoLib.ServoStep(rfserv, 0));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(2), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false)); //90d left
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(24), true));



        //TODO: Actually Make this
        //sudo code

        //move forward 1.5 tiles (pos 1)
        //move left until find skystone
        //grab skystone
        //move back to pos 1

        //func 1{
        //move backward 0.5 tile
        //move right 1.5t
        //drop skysotne
        //move left 1.5t}

        //move forward 0.5t (pos 1)
        //move left until find skystone
        //grab skystone
        //return to pos 1

        //func 1() but only move left 1t at end

        done = false;
        //end sequence

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
    @Override
    public void start(){

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

    @Override
    public void stop(){
        super.stop();
    }
}
