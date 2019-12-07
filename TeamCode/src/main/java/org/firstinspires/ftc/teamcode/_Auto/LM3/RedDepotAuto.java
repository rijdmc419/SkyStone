package org.firstinspires.ftc.teamcode._Auto.LM3;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java
@Autonomous(name="USE THIS RED DEPOT AUTO")
public class RedDepotAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    ColorSensor clrSnr;
    DistanceSensor dstSnr;
    DcMotor motors[];
    AutoLib.Sequence seq;
    Servo serv;

    boolean done;

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

        motors = new DcMotor[4];
        robot.init(hardwareMap);

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

        seq = new AutoLib.LinearSequence();

        //Align with the first stone
        //Servo up
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(29), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));//turns left
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(4), false));

        //Start testing loop
        while(/*Stone.color &&*/ dstSnr.getDistance(DistanceUnit.INCH) < 10){
            seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(4), false));
            i++;
        }
        //swerve left 7"
        //servo down / intake grab
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-4*i), false));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-36), false));
        //servo up / intake ungrab
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(12), false));

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
    @Override
    public void stop(){
        super.stop();
    }
}
