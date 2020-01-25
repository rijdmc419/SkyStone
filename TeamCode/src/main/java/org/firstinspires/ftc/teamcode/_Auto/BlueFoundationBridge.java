package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;


@Autonomous(name = "Blue Foundation Complex Bridge Park")
public class BlueFoundationBridge extends OpMode{
    SkystoneHardware robot = new SkystoneHardware();
    DcMotor motors[];
    Servo lfserv, rfserv;
    BNO055IMUHeadingSensor imu;
    AutoLib.Sequence seq;
    SensorLib.PID pid;
    boolean done = false;

    //@Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        lfserv = robot.lfServo;
        rfserv= robot.rfServo;

        imu = robot.imu;

        float Kp = 0.02f;
        float Ki = 0.025f;
        float Kd = 0f;
        float KiCutoff = 10f;
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        float uniPow = 0.33f;

        seq = new AutoLib.LinearSequence();

        //TODO: Code the thing

        //Start
        //Move to Foundation
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this,0f, 0f, imu, pid, motors, uniPow, travDist(12f), false)); //0.5Tile(T) east
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, -90f, 0f, imu, pid, motors, uniPow, travDist(24f), false )); //1T north
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this,0f, 0f,imu,pid,motors,uniPow,travDist(24f),false));//1T east

        //grab  the thing
        seq.add(new AutoLib.ServoStep(lfserv,1f,1f));//put left foundation servo down //TODO: FixNums
        seq.add(new AutoLib.ServoStep(rfserv, 0f,1f));//put right foundation servo down //TODO: FixNums
        seq.add(new AutoLib.TimedMotorStep(motors[0],0f,1.5f,false));//waits for servos to finish moving

        //pull back to build site
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 180f, 0f, imu, pid, motors, uniPow, travDist(36f), false));//1.5T west

        //Release the thing
        seq.add(new AutoLib.ServoStep(lfserv,0f,1f));//put left foundation servo up //TODO: FixNums
        seq.add(new AutoLib.ServoStep(rfserv, 1f,1f));//put right foundation servo up //TODO: FixNums
        seq.add(new AutoLib.TimedMotorStep(motors[0],0f,1.5f,false));//waits for servos to finish moving

        //navigate to tape (bridge side)
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 0f, imu, pid, motors, uniPow, travDist(24f), false));//1T South
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 0f, 0f, imu, pid, motors, uniPow, travDist(24f), false));//1T East
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 0f, imu, pid, motors, uniPow, travDist(24f), true));//1T South
        //parked on tape end

        done = false;
    }

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }


    //@Override
    public void start(){

    }

    public void loop(){
        if (!done){
            done = seq.loop(); // returns true when we're done
        }
        else{
            telemetry.addData("Sequence finished", "");
        }
    }
}
