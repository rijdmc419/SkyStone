package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name="LM1 Foundation Side")
public class FoundationSideAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware(); //get the robots hardware
    ColorSensor clrSnr;
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
        BNO055IMU.Parameters gParams = new BNO055IMU.Parameters();
        gParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParams.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;

        clrSnr = robot.clrSnr; //see the above comment

        double inTravel = 560/(4*(Math.PI)); //this should convert to inches
        float uniPow = 0.33f; //for 20:1 motors
        float inTile = 24f; //amount of inches in tile use for when need to travel by number of tiles

        //560 == 1 rotation of the wheel (I think?)
        //Which should be around 4pi inches or ~12.56637 inches
        seq = new AutoLib.LinearSequence();
        //TODO: Make an Auto
        //seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, (int) Math.round(6*inTravel), true)); //should travel 6 in
        //seq.add(new AutoLib.AzimuthDistanceDriveStep(this, -90f, x, y, motors, uniPow, d,5f)); //maybe mark with the inTravel thing
        //seq.add(new AutoLib.SquirrelyGyroCountedDriveStep) //TODO: Get this to work, or see if there is something better for strafing
                                                             //TODO: Also setup a PID Loop and figure out how to use Gyro

        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, (int) Math.round(1.5f*inTile*inTravel), false)); //move forward 1.5 tiles
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, -560, 560, true));//rotate 90deg right
        //move forward 1 tile
        //rotate 90d left
        //move forward .5 tile
        //foundation grabber grab
        //move backward 1.5t (sub to change)
        //un-grab
        //rotate left 90d
        //move forward 2 tiles

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
            telemetry.addData("Gyro: ", gyr0.getTemperature());
            telemetry.addData("ClrSnr: ", clrSnr.argb());
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