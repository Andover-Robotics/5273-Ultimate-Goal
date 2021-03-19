package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import java.util.Timer;
import java.util.TimerTask;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp(
        name = "Shooter Self-Tuner",
        group = "AA"
)
public final class ShooterPIDFSelfTuner extends OpMode {
    private DcMotorEx flywheel;

    private FtcDashboard dash;
    private GamepadEx pad;

    public static int targetRPM = GlobalConfig.SHOOTER_RPM;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(GlobalConfig.SHOOTER_PIDF_COEFFICIENTS);
    public static double errorFMultiplier = 0.01;

    // Estimated value obtained via observation - how many current RPM readings are plotted per second
    public static final double RPM_READINGS_PER_SECOND = 75;
    // How long, in seconds, each RPM rolling average duration should be
    public static double RPMRollingAverageDuration = 0.5;

    // Used for rolling average calculation
    private double currentRPMSum = 0, latestRPMRollingAverage = 0;

    // Stores the latest RPM readings. Used to implement a sliding window algorithm for latestRPMRollingAverage
    private double[] RPMReadings = new double[(int) (RPMRollingAverageDuration * RPM_READINGS_PER_SECOND + 0.5)];

    // Counters:
    //      currentIndex: the current index in RPMReadings to add the next RPM Reading to
    //      numItems: how many items to divide by in the rolling average calculation (will cap out at RPMReadings.length and stop incrementing)
    //      readingsTaken: how many RPM readings have been taken since the call to begin recording and calculating the next RPM Rolling Average has been made
    private int currentIndex = 0, numItems = 0, readingsTaken = 0;

    // Is true when readingsTaken should be incremented and a new rolling average should be calculated
    // This was added so that time can be given for stabilization after every change to F, while
    // excluding the readings from this period from the rolling average calculation
    private boolean countingReadings = false;

    // Configurable delays: how long to wait before performing the first rolling average and how long to wait between future rolling averages (in milliseconds)
    public static int firstRollingAverageDelay = 4000, rollingAverageDelay = 600;

    // The value of getRunTime() on start()
    private double startTime;

    public void init() {
        this.flywheel = this.hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dash = FtcDashboard.getInstance();
        pad = new GamepadEx(gamepad1);
    }

    @Override
    public void start() {
        startTime = getRuntime();
        // Begin counting for the rolling average on time
        Timer startAverageTimer = new Timer();
        startAverageTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                countingReadings = true;
            }
        }, firstRollingAverageDelay);
    }

    public void loop() {
        if (pad.wasJustPressed(DPAD_UP)) {
            targetRPM += 100;
        } else if (pad.wasJustPressed(DPAD_DOWN)) {
            targetRPM -= 100;
        } else if (pad.wasJustPressed(DPAD_LEFT)) {
            targetRPM += 50;
        } else if (pad.wasJustPressed(DPAD_RIGHT)) {
            targetRPM -= 50;
        }

        applyShootRpm();

        double currentRPM = flywheel.getVelocity() * 60.0 / 28.0;

        // Implement sliding window algorithm to efficiently manage currentRPMSum
        currentRPMSum -= RPMReadings[currentIndex];
        RPMReadings[currentIndex] = currentRPM;
        currentRPMSum += RPMReadings[currentIndex];

        // Increment numItems until it hits its cap
        numItems = Math.min(RPMReadings.length, numItems + 1);

        // Increment currentIndex, wrapping back to the first element after writing to the last
        currentIndex = (currentIndex + 1) % RPMReadings.length;

        // If we are currently trying to calculate a rolling average, begin the counter for how
        // much data has been taken for this calculation
        readingsTaken += countingReadings ? 1 : 0;

        // Check if the current rolling average calculation has enough NEW data to be performed
        if (readingsTaken >= RPMReadings.length) {
            // Calculate the rolling average with the latest data
            latestRPMRollingAverage = currentRPMSum / numItems;

            // Reset the flag and the counter for the next calculation, setting up to allow the motor
            // to have time to adjust itself without impacting the next rolling average
            readingsTaken = 0;
            countingReadings = false;

            // Adjust the F coefficient based off of the latest rolling average
            double error = targetRPM - latestRPMRollingAverage;
            pidfCoefficients.f += error * errorFMultiplier;
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // Schedule the next rolling average calculation (so that spinup time is allotted)
            Timer startAverageTimer = new Timer();
            startAverageTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    countingReadings = true;
                }
            }, rollingAverageDelay);
        }

        TelemetryPacket plottedData = new TelemetryPacket();

        plottedData.put("Current RPM", currentRPM);
        plottedData.put("Target RPM", targetRPM);
        plottedData.put("Latest Rolling Average", latestRPMRollingAverage);
        plottedData.put("Current F Coefficient", pidfCoefficients.f);

        dash.sendTelemetryPacket(plottedData);

        // Send data to the Driver Station
        telemetry.addLine("Use the D-Pad to adjust the target RPM.");
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Latest Rolling Average", latestRPMRollingAverage);
        telemetry.addData("Current F Coefficient", pidfCoefficients.f);
        telemetry.addData("Time Running", "%.2f Seconds", getRuntime() - startTime);

        telemetry.update();
    }

    public void stop() {
        flywheel.setVelocity(0);
    }

    public final void applyShootRpm() {
        flywheel.setVelocity(28.0 * targetRPM / 60.0);
    }

}