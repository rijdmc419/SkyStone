package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
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


@Autonomous(name = "Blue Foundation Complex Bridge Park")
public class BlueFoundationBridge extends OpMode{

    SkystoneHardware robot = new SkystoneHardware(); //the robot
    DcMotor motors[]; // array of motors
    Servo lfserv, rfserv; //foundation mover servos
    BNO055IMUHeadingSensor imu; //heading sensor / gyro

    SensorLib.EncoderGyroPosInt posInt; //Encoder/gyro-based position integrator to keep track of where we are

    static int botLength = 16;
    static int tl = 24; //tile length (in inches)
    int tol = 1; //tolerance for error (in inches)

    AutoLib.Sequence seq; //autoLib sequence

    SensorLib.PID pid; //the PID

    float uniPow = 0.33f; //universal power for motors

    boolean done = false;

    //@Override
    public void init(){
        motors = new DcMotor[4];
        robot.init(hardwareMap); //VERY IMPORTANT DO NOT REMOVE (so I am told)

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

        //uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side for coordinate system
        // our code uses bearing zero = +Y,
        Position initPos = new Position(DistanceUnit.INCH, 1*tl, (3*tl) - botLength/2, 0.0, 0); // at the red wall f-side
        posInt = new SensorLib.EncoderGyroPosInt(SensorLib.EncoderGyroPosInt.DriveType.MECANUM, this, imu, motors, 560, 4, initPos);

        seq = new AutoLib.LinearSequence();

        //TODO: Code the thing

        //Start
        //move to foundation
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 2 * tl, (-1 * tl) - botLength/2, 0, 0), 180, tol, false)); //move to (2,-1)

        //grab  the thing
        seq.add(new AutoLib.ServoStep(rfserv, 0f, 1f)); //right servo down TODO: FixNums
        seq.add(new AutoLib.ServoStep(lfserv, 1f, 1f)); //left servo downs TODO: FixNums
        seq.add(new AutoLib.LogTimeStep(this, "waiting for servos", 1.5));


        //pull back to build site
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -2 * tl, (3 * tl) - botLength/2, 0, 0), 180, tol, false)); //move to (2,-1)


        //release the thing
        seq.add(new AutoLib.ServoStep(rfserv, 1f, 1f)); //right servo down TODO: FixNums
        seq.add(new AutoLib.ServoStep(lfserv, 0f, 1f)); //left servo downs TODO: FixNums
        seq.add(new AutoLib.LogTimeStep(this, "waiting for servos", 1.5));

        //navigate to tape (bridge side)
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl, (3 * tl) - botLength/2, 0, 0), 0, tol, false));
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -1 * tl, (1.5 * tl), 0, 0), 180, tol, false)); //move to (2,-1)

        //parked on tape
        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, 0, (1.5 * tl), 0, 0), 180, tol, true));

        //End

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
