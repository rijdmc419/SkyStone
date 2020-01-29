package org.firstinspires.ftc.teamcode._Auto.DepotSideAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Disabled
@Autonomous(name = "Complex Auto Test")
public class ComplexTest extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    BNO055IMUHeadingSensor imu;
    SensorLib.PID pid;
    SensorLib.EncoderGyroPosInt posInt;    // Encoder/gyro-based position integrator to keep track of where we are
    DcMotor motors[];
    int botLength = 18; //TODO: FixNum
    int tl = 24; //tile length (in inches)
    int tol = 1; //tolerance for error (in inches)
    //INSERT MOTORS, SERVOS, AND SENSORS HERE
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 1f;

    @Override
    public void init(){
        bot.init(hardwareMap);

        imu = bot.imu;

        //pid setup stuff
        float Kp = 0.001f; //TODO: FixNum       // motor power proportional term correction per degree of deviation
        float Ki = 0.01f;         // ... integrator term
        float Kd = 0;      // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        Position initPos = new Position(DistanceUnit.INCH, -1*tl, (3*tl)-botLength/2, 0.0, 0); // at the BLUE wall
        posInt = new SensorLib.EncoderGyroPosInt(SensorLib.EncoderGyroPosInt.DriveType.MECANUM, this, imu, motors, 560, 4, initPos);

        motors = new DcMotor[4];
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        //INSERT MOTORS, SERVOS, AND SENSORS HERE
        //uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side for coordinate system
        // our code uses bearing zero = +Y,
        seq = new AutoLib.LinearSequence();
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1*tl, 2*tl, 0, 0), -90, tol, true));

    }
    public int travDist(float in){
        double c = 560/(4*(Math.PI));
        int out = (int) Math.round(c*in);
        return out;
    }
    public int lRot(float deg){ //- = counterclockwise, + = clockwise
        float c = -9.474f;
        float out = c*deg;
        return  Math.round(out);
    }
    public int rRot(float deg){ //- = counterclockwise, + = clockwise
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
