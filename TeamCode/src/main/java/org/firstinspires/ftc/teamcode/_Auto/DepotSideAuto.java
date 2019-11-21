package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;
//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java
@Autonomous(name="LM1 Depot Side")
public class DepotSideAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];

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

        //start sequence
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(36), false));
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 90f, uniPow, 0.5f, true));

        //TODO: Actually Make this
        //sudo code
        //move forward 1.5 tiles (pos 1)
        //move left until fine skystone
        //grab skystone
        //move back to pos 1

        done = false;
        //end sequence

    }

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
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
