package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name="LM1 Foundation Side")
public class FoundationSideAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware(); //get the robots hardware
    DcMotor motors[];
    BNO055IMU gyr0;
    AutoLib.Sequence seq;
    boolean done;

    @Override
    public void init(){
        motors =new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl; //makes motors

        gyr0 = robot.gyr0; //i got no clue what this does or if it works

        double inTravel = 560/(4*(Math.PI)); //this should convert to inches
        float uniPow = 0.33f; //for 20:1 motors
        //560 == 1 rotation of the wheel (I think?)
        //Which should be around 4pi inches or ~12.56637 inches
        seq = new AutoLib.LinearSequence();
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, (int) Math.round(6*inTravel), true)); //should travel 6 in
        //seq.add(new AutoLib.AzimuthDistanceDriveStep(this, -90f, x, y, motors, uniPow, d,5f)); //maybe mark with the inTravel thing
        //seq.add(new AutoLib.SquirrelyGyroCountedDriveStep) //TODO: Get this to work, or see if there is something better for strafing
                                                             //TODO: Also setup a PID Loop and figure out how to use Gyro
        done = false;
    }

    @Override
    public void start(){
        telemetry.addData("Starting Auto", "");

    }

    @Override
    public void loop(){
        if (!done){
            done = seq.loop(); // returns true when we're done
            telemetry.addData("Gyro: ", gyr0);
        }
        else{
            telemetry.addData("Sequence finished", "");
        }

    }

    @Override
    public void stop(){
        super.stop();
    } //stop but more
}
