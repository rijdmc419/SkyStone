package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Disabled
@Autonomous(name = "Test")
public class AutoTemplate extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    BNO055IMUHeadingSensor imu;
    SensorLib.PID pid;
    DcMotor motors[];
    //INSERT MOTORS, SERVOS, AND SENSORS HERE
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 1f;

    @Override
    public void init(){
        bot.init(hardwareMap);

        imu = bot.imu;

        //pid setup stuff
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        motors = new DcMotor[4];
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        //INSERT MOTORS, SERVOS, AND SENSORS HERE

        seq = new AutoLib.LinearSequence();

        //INSERT SEQUENCE HERE
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
