package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;
//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java
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
        //560 == 1 rotation of the wheel (I think?)
        //Which should be around 4pi inches or ~12.56637 inches
        seq = new AutoLib.LinearSequence();
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(36), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(-90), rRot(-90), false));
        seq.add(new AutoLib.MoveByEncoderStep(motors, uniPow, travDist(24), false));
        seq.add(new AutoLib.TurnByEncoderStep(motors[0], motors[1], motors[2], motors[3], uniPow, uniPow, lRot(90), rRot(90), true));

        //seq.add(new AutoLib.AzimuthDistanceDriveStep(this, -90f, x, y, motors, uniPow, d,5f)); //maybe mark with the inTravel thing
        //seq.add(new AutoLib.SquirrelyGyroCountedDriveStep) //TODO: Get this to work, or see if there is something better for strafing
                                                                //TODO: Also setup a PID Loop and figure out how to use Gyro
        done = false;
    }

    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }
    public int lRot(float deg){ //pos to counterclockwise, neg to clockwise
        float c = -9.474f;
        float out = c*deg;
        return (int) out;
    }
    public int rRot(float deg){ //pos to counterclockwise, neg to clockwise
        float c = 9.648f;
        float out = c*deg;
        return (int) out;
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
