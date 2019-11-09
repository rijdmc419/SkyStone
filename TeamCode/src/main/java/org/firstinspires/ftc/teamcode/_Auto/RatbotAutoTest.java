package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name="Sksystone Ratbot Test")
public class RatbotAutoTest extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();


    AutoLib.Sequence seq;
    boolean done;
    DcMotor motors[];



    @Override
    public void init(){
        AutoLib.HardwareFactory mf = null;
        mf = new AutoLib.RealHardwareFactory(this);
        robot.init(hardwareMap);

        motors = new DcMotor[4];
        motors[0] = robot.fr;
        motors[1] = robot.fl;
        motors[2] = robot.bl;
        motors[3] = robot.br;

        float power = 0.3f;
        float sec = 2f;

        seq = new AutoLib.LinearSequence();
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 0f, power, sec, false));
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 180f, power, sec, false));
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, -90f, power, sec, false));
        seq.add(new AutoLib.MoveSquirrelyByTimeStep(motors, 90f, power, sec, true));

        done = false;
    }

    @Override
    public void start(){

    }

    @Override
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
