package org.firstinspires.ftc.teamcode._Auto.DepotSideAuto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name = "Stone Grab Test (Red Depot Bridge)")
public class RedDepotSideFar extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    DcMotor[] motors;
    Servo tserv;
    BNO055IMUHeadingSensor imu;

    AutoLib.Sequence seq;
    boolean done;
    float uniPow =0.33f;
    SensorLib.PID pid;

    @Override
    public void init(){
        motors = new DcMotor[4];
        bot.init(hardwareMap);

        //motors init
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        //servos init
        tserv = bot.tempServo;

        imu = bot.imu;

        //pid setup stuff
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        //sequence init
        seq = new AutoLib.LinearSequence();
     //   seq.add(new AutoLib.AzimuthCountedDriveStep(this, 0, imu, pid , motors, uniPow, travDist(24*4), true));
        //TODO: arm up & moves to stone
        //seq.add(new AutoLib.ServoStep(tserv,0f)); //moves servo up //TODO: Fix
       // seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 5, false));
        seq.add(new AutoLib.AzimuthCountedDriveStep(this, 0, imu, pid , motors, 1f, travDist(24), false));
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 5, false)); //this is for debug
        //heading == swerve direction, orientation == rotation
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 90f, imu, pid, motors, 1f, travDist(24), true)); //why this line no work :(
    //    seq.add(new AutoLib.AzimuthCountedDriveStep(this, -90f, imu, pid , motors, uniPow, travDist(8), true));

        //TODO: arm down
        //TODO: move back to foundation side
        //TODO: arm up
        //TODO: move back to get a second stone
        //TODO: arm down
        //TODO: move back to foundation side
        //TODO: arm up
        //TODO: park on tape (closest to skyBridge)
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
        telemetry.addData("IMU Angle", imu.getHeading());
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
