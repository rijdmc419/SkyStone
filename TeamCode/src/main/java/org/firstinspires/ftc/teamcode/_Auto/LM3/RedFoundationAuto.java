package org.firstinspires.ftc.teamcode._Auto.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java

@Autonomous(name="Simple Red Foundation")
public class RedFoundationAuto extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    ColorSensor clrSnr;
    DcMotor motors[];
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 0.33f;

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }

    @Override
    public void init(){
        motors = new DcMotor[4];
        bot.init(hardwareMap);
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        clrSnr = bot.clrSnr;

        seq = new AutoLib.LinearSequence();
    }

    @Override
    public void start(){
        telemetry.addData("Starting Auto","");
    }

    @Override
    public void loop(){
        if (!done){
            done = seq.loop(); // returns true when we're done
            //   telemetry.addData("ClrSnr: ", clrSnr.argb());
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
