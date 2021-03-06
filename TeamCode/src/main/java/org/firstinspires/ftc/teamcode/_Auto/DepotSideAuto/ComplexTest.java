package org.firstinspires.ftc.teamcode._Auto.DepotSideAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

@Autonomous(name = "Complex Auto Test (Blue Depot)")
public class ComplexTest extends OpMode {
    SkystoneHardware bot = new SkystoneHardware();
    BNO055IMUHeadingSensor imu;
    SensorLib.PID pid;
    SensorLib.EncoderGyroPosInt posInt;    // Encoder/gyro-based position integrator to keep track of where we are
    DcMotor motors[];
    Servo serv;
    int botLength = 16;
    int stoneWidth = 4;
    int tl = 24; //tile length (in inches)
    int tol = 2; //tolerance for error (in inches)
    //INSERT MOTORS, SERVOS, AND SENSORS HERE
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 0.33f;

    @Override
    public void init() {
        bot.init(hardwareMap);

        imu = bot.imu;
        imu.setHeadingOffset(180f);

        //pid setup stuff
        float Kp = 0.01f; //TODO: FixNum (oscillates just a little bit on swerving) // motor power proportional term correction per degree of deviation
        float Ki = 0f;         // ... integrator term
        float Kd = 0f;      // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        motors = new DcMotor[4];
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;

        for (int i=0; i<motors.length; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Position initPos = new Position(DistanceUnit.INCH, -1 * tl, (3 * tl) - (botLength / 2), 0.0, 0); // at the BLUE wall
        posInt = new SensorLib.EncoderGyroPosInt(SensorLib.EncoderGyroPosInt.DriveType.MECANUM, this, imu, motors, 560, 4, initPos);

        //INSERT MOTORS, SERVOS, AND SENSORS HERE
        //uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side for coordinate system
        // our code uses bearing zero = +Y, clockwise = -, counterClockwise = +
        //bearing is absolute
        seq = new AutoLib.LinearSequence();
        //goes to 1st stone and grabs it
        seq.add(new AutoLib.ServoStep(serv, 0f, 1f)); //Arm up
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 2, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl, (1*tl), 0, 0), 180, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl -(1*stoneWidth), 1 * tl + (botLength / 2), 0, 0), 90, tol, false));
     //   seq.add(new AutoLib.ServoStep(serv, 1f, 1f)); //Arm down
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 2,false)); //sstone grabbed, ready to cross bridge
        //crosses bridge & deposits stone
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl, 2 * tl, 0, 0), 90, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 1 * tl, 2 * tl, 0, 0), 0, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 1 * tl, 2.5 * tl, 0, 0), 0, tol, false));
     //   seq.add(new AutoLib.ServoStep(serv, 0f, 1f)); //Arm up
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 2, false));
        //goes back and grabs second stone
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 1 * tl, 2 * tl, 0, 0), 0, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl -(2*stoneWidth), 1 * tl + (botLength / 2), 0, 0), 90, tol, false));
    //    seq.add(new AutoLib.ServoStep(serv, 1f, 1f)); //Arm down
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 2,false)); //sstone grabbed, ready to cross bridge
        //crosses bridge & deposits stone
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl, 2 * tl, 0, 0), 90, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 1 * tl, 2 * tl, 0, 0), 0, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 1 * tl, 2.5 * tl, 0, 0), 0, tol, false));
     //   seq.add(new AutoLib.ServoStep(serv, 0f, 1f)); //Arm up
        seq.add(new AutoLib.TimedMotorStep(motors[0], 0, 2, false));
        //parks on tape
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 0 * tl, 1.5 * tl, 0, 0), -90, tol, true));

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
