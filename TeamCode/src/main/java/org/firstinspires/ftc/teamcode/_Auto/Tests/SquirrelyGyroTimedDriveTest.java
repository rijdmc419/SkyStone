package org.firstinspires.ftc.teamcode._Auto.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name = "SquirrelyGyroTimedDriveTest")
public class SquirrelyGyroTimedDriveTest extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    DcMotor motors[];
    Servo serv;

    BNO055IMUHeadingSensor imu;
    SensorLib.PID pid;

    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 0.33f;

    @Override
    public void init(){
        float tleg = 1.5f;
        bot.init(hardwareMap);

        motors = new DcMotor[4];

        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        serv = bot.tempServo;

        imu = bot.imu;

        //pid setup stuff
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        seq = new AutoLib.LinearSequence();

        // drive a square while maintaining constant orientation (0)
        seq.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, -90,0,imu, pid, motors, uniPow, tleg/2,false));
        seq.add(new AutoLib.ServoStep(serv, 0f, 1f));
        seq.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0,imu, pid, motors, uniPow, tleg, false));
        seq.add(new AutoLib.ServoStep(serv, 1f, 1f));
        seq.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 90, 0,imu, pid, motors, uniPow, tleg, false));
        seq.add(new AutoLib.ServoStep(serv, 0f, 1f));
        seq.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 180, 0,imu, pid, motors, uniPow, tleg, false));
        seq.add(new AutoLib.ServoStep(serv, 1f, 1f));
        seq.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 270, 0,imu, pid, motors, uniPow, tleg/2, false));

        /* What Paul wrote:
        *
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, -90, 0, rh.mIMU, pid, rh.mMotors, power, leg/2, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this,   0, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this,  90, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 180, 0, rh.mIMU, pid, rh.mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 270, 0, rh.mIMU, pid, rh.mMotors, power, leg/2, false));
        */
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
