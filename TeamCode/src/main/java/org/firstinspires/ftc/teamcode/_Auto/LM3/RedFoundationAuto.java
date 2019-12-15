package org.firstinspires.ftc.teamcode._Auto.LM3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java

@Autonomous(name="LM3 Simple Red Foundation")
public class RedFoundationAuto extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    ColorSensor lClr;
    Servo lfServ, rfServ;
    DcMotor motors[];
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 0.33f;
    public RedFoundationAuto(){
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
        bot.init(hardwareMap);

        //motors
        motors = new DcMotor[4];
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        //servos
        lfServ = bot.lfServo;
        rfServ = bot.rfServo;

        //sensors
        lClr = bot.lClr;

        seq = new AutoLib.LinearSequence();

        //Move to foundation
        seq.add(new AutoLib.ServoStep(lfServ, 0)); //TODO: Test to see that this actually goes up
        seq.add(new AutoLib.ServoStep(rfServ,1));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(24), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(90), rRot(90), false));//turns right
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(24), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));//turns left
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(2), false));
        //Lower Servos
        seq.add(new AutoLib.ServoStep(lfServ, 0));
        seq.add(new AutoLib.ServoStep(rfServ,1 ));
        seq.add(new AutoLib.MoveByTimeStep(motors, 0, 1.5, false)); //wait for a bit
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(12), false));
        seq.add(new AutoLib.ServoStep(lfServ, 1));
        seq.add(new AutoLib.ServoStep(rfServ,0));
        //wait & pull foundation back
        seq.add(new AutoLib.MoveByTimeStep(motors, 0, 2, false)); //wait for a bit
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(-24), false));
        //Raise Servos
        seq.add(new AutoLib.ServoStep(lfServ, 0));
        seq.add(new AutoLib.ServoStep(rfServ,1));
        seq.add(new AutoLib.MoveByTimeStep(motors, 0, 1.5, false)); //wait for a bit

        //Go back to tape (close to wall)
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));//turns left
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(55), true));
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
