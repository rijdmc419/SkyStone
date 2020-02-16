package org.firstinspires.ftc.teamcode._Auto.ParkOnTape;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;
//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java

@Autonomous(name="RED Foundation Park Close")
public class FoundationSideAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    //Servo lfserv, rfserv;
    AutoLib.Sequence seq;
    boolean done;

    @Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;
        float uniPow = 0.33f;
        seq = new AutoLib.LinearSequence();

        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(2), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));//turns right
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(24), true));

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
    public void stop(){ super.stop(); }
}
