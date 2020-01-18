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
        //robot.init(hardwareMap);

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

        //TODO: WHY IT NO WORK VV
        //telemetry.addData("Left Servo",lfserv.getPosition());
        //telemetry.addData("Right Servo",rfserv.getPosition());

        seq = new AutoLib.LinearSequence();

        //TODO: Code the thing

        //Pseudo Code
        //Start
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this,0f, 0f, imu, pid, motors, uniPow, travDist(12f), false)); //0.5Tile(T) east
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, -90f, -90f, imu, pid, motors, uniPow, travDist(24f), false )); //1T north
        seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this,0f, 0f,imu,pid,motors,uniPow,travDist(24f),true));//1T east
        //grab  the thing
        //1.5T west
        //Release the thing
        //1T South
        //1T East
        //1T South

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
            //telemetry.addData("Sequence finished", ""); //TODO: Telemetry is borked for some reason
        }
    }
}
