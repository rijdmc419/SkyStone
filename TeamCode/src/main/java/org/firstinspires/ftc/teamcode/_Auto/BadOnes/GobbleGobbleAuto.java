package org.firstinspires.ftc.teamcode._Auto.BadOnes;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode._Libs.AutoLib;
        import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
        import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
        import org.firstinspires.ftc.teamcode._Libs.SensorLib;
        import org.firstinspires.ftc.teamcode._Libs.hardware.SkystoneHardware;

//Useful Thing:
//https://github.com/Scott3-0/7776-ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/old2017-18/UltraAuto.java
@Disabled
@Autonomous(name="PID Tester Broken")
public class GobbleGobbleAuto extends OpMode {
    SkystoneHardware robot = new SkystoneHardware();
    ColorSensor clrSnr;
    DcMotor motors[];
    BNO055IMUHeadingSensor imu;
    AutoLib.Sequence seq;
    SensorLib.PID mPID = new SensorLib.PID(0.02f, 0.025f, 0, 10); //TODO: TUNE THIS BITCH
    SensorLib.PIDAdjuster mPIDAdjuster;

    boolean bSetup;
    boolean done;

    public GobbleGobbleAuto(){
        this.msStuckDetectInit = 10000;
    }

    @Override
    public void init(){
        motors =new DcMotor[4];
        robot.init(hardwareMap);

        motors[0] = robot.fr;
        motors[1] = robot.br;
        motors[2] = robot.fl;
        motors[3] = robot.bl;

        clrSnr = robot.lClr;

        imu = robot.imu;

        float initHeading = 0f;
        imu.setHeadingOffset(initHeading);

        float uniPow = 0.17f;

        mPIDAdjuster = new SensorLib.PIDAdjuster(this, mPID, gamepad1);

        seq = new AutoLib.LinearSequence();

      //  seq.add(new AutoLib.AzimuthCountedDriveStep(this, 0f, imu, mPID, motors, uniPow, travDist(24), false));
        //seq.add(new AutoLib.AzimuthCountedDriveStep(this, 90f, imu, mPID, motors, uniPow, travDist(24), false));
        //seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 0f, 0f, imu, mPID, motors, uniPow, travDist(36), true));
        //seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 0f, imu, mPID, motors, uniPow, travDist(36)))
       // seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 90f, imu, mPID, motors, uniPow, travDist(24), true));
       // seq.add(new AutoLib.AzimuthTimedDriveStep(this, 0f, imu, mPID, motors, uniPow, 30, true));

       /* seq.add(new AutoLib.AzimuthTimedDriveStep(this, 0f, imu, mPID, motors, uniPow, 5f, false));
        seq.add(new AutoLib.AzimuthTimedDriveStep(this, 90f, imu, mPID, motors, uniPow, 5f, false));
        seq.add(new AutoLib.AzimuthTimedDriveStep(this, 180f, imu, mPID, motors, uniPow, 5f, false));
        seq.add(new AutoLib.AzimuthTimedDriveStep(this, 270f, imu, mPID, motors, uniPow, 5f, false)); */

       seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 0f, 0f, imu, mPID, motors, uniPow,  travDist(24), false));
       seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 90f, 90f, imu, mPID, motors, uniPow,  travDist(24), false));
       seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 180f, 180f, imu, mPID, motors, uniPow,  travDist(24), false));
       seq.add(new AutoLib.SquirrelyGyroCountedDriveStep(this, 270f, 270f, imu, mPID, motors, uniPow,  travDist(24), true));

        done = false;
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
        try{
            if (gamepad1.y)
                 bSetup = true;      // Y button: enter "setup mode" using controller inputs to set Kp and Ki
            if (gamepad1.x)
                 bSetup = false;     // X button: exit "setup mode"
             if (bSetup) {           // "setup mode"
                mPIDAdjuster.loop();
                return;
            }
          telemetry.update();

        } catch(NullPointerException e){
            telemetry.addData("PID Adjuster Didn't work","");
            telemetry.update();
        }

        //if (!done){
            seq.loop(); // returns true when we're done
         //   telemetry.addData("ClrSnr: ", clrSnr.argb());
        //}
        //else{
            //telemetry.addData("Sequence finished", "");
        //}
        telemetry.addData("IMU Heading", imu.getHeading());

    }

    @Override
    public void stop(){
         super.stop();
    }
}
