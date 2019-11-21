package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;

@Autonomous(name="depot league meet cero")
public class LM0Auto extends OpMode {
    AutoLib.Sequence seq;
    boolean done;
    DcMotor motors[];

    public void init(){
        motors = new DcMotor[4];

        motors[0] = hardwareMap.dcMotor.get("fr");
        motors[1] = hardwareMap.dcMotor.get("br");
        motors[2] = hardwareMap.dcMotor.get("fl");
        motors[3] = hardwareMap.dcMotor.get("bl");

        float power = 0.5f;
        float sec = 0.5f;

        seq = new AutoLib.LinearSequence();
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 0f, power, 0.25f, false));
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 90f, power, sec, true));

        done = false;
    }
    public void start(){
        telemetry.addData("bruh","Eat Shit");
    }

    public void loop(){
        if (!done)
            done = seq.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
    }
}
