package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_SkyStone;

/**
 * Created by phanau on 9/14/18.
 */

/**
 * This OpMode illustrates the basics of using the 2019-20 VuforiaLib_SkyStone library to determine
 * positioning and orientation of robot on the FTC field.
 */

@Autonomous(name="Test: Vuforia Navigation Test SkyStone", group ="Test")
//@Disabled
public class VuforiaNavigationTestSkyStone extends OpMode {

    VuforiaLib_SkyStone mVLib;

    public VuforiaNavigationTestSkyStone()
    {
        // override default init timeout to prevent timeouts while starting Vuforia on slow phones.
        // need to do it here so it's in effect BEFORE init() is called.
        this.msStuckDetectInit = 10000;
    }

    @Override public void init() {
        /**
         * Start up Vuforia
         */
        mVLib = new VuforiaLib_SkyStone();
        mVLib.init(this);     // pass it this OpMode (so it can do telemetry output)
    }

    @Override public void start()
    {
        /** Start tracking the data sets we care about. */
        mVLib.start();
    }

    @Override public void loop()
    {
        mVLib.loop(false);       // update location info

        String allVisible = new String();
        for (String visible : mVLib.getVisibleNames())
            allVisible = allVisible + visible + ",";
        telemetry.addData("Visible", allVisible);

        if (mVLib.haveLocation()) {
            telemetry.addData("Position", mVLib.formatPosition(mVLib.getLastLocation()));
            telemetry.addData("Field Position", mVLib.getFieldPosition());
        }
        else
            telemetry.addData("Position", "Unknown");

        if (mVLib.haveHeading()) {
            telemetry.addData("Orientation", mVLib.formatOrientation(mVLib.getLastLocation()));
            telemetry.addData("Heading", mVLib.getHeading());
        }
        else
            telemetry.addData("Orientation:", "Unknown");

    }

    @Override public void stop()
    {
        mVLib.stop();
    }

}
