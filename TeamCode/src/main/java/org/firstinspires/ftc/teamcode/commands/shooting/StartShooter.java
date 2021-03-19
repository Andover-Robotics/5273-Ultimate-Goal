package org.firstinspires.ftc.teamcode.commands.shooting;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.teleop.ShooterPIDFSelfTuner;

import java.util.Timer;
import java.util.TimerTask;


public class StartShooter extends CommandBase {
    private ShooterSubsystem shooter;

    // Used for rolling average calculation
    private double currentRPMSum = 0, latestRPMRollingAverage = 0, rollingAverageLengthSeconds = 0.25, acceptableRPMThreshold = 50;

    // Stores the latest RPM readings. Used to implement a sliding window algorithm for latestRPMRollingAverage
    private double[] RPMReadings = new double[(int) (rollingAverageLengthSeconds * ShooterPIDFSelfTuner.RPM_READINGS_PER_SECOND + 0.5)];

    // Counters:
    //      currentIndex: the current index in RPMReadings to add the next RPM Reading to
    //      numItems: how many items to divide by in the rolling average calculation (will cap out at RPMReadings.length and stop incrementing)
    //      readingsTaken: how many RPM readings have been taken since the call to begin recording and calculating the next RPM Rolling Average has been made
    //      numConsecutive: how many consecutive values of latestRollingAverage have currently been found to be within the acceptableRPMThreshold in a row
    //      numConsecutiveRequired: how many consecutive values of latestRollingAverage must be within the acceptableRPMThreshold to be finished
    //      rollingAverageDelay: how many milliseconds to wait in between the completion of one rolling average and the beginning of the next
    private int currentIndex = 0, numItems = 0, readingsTaken = 0, numConsecutive, numConsecutiveRequired = 2, rollingAverageDelay = 25;

    // Is true when readingsTaken should be incremented and a new rolling average should be calculated
    // This was added so that time can be given for stabilization after every change to F, while
    // excluding the readings from this period from the rolling average calculation
    private boolean countingReadings = false;

    private Telemetry telemetry;

    public StartShooter(ShooterSubsystem shooter, Telemetry telemetry) {
        this.shooter = shooter;
        this.telemetry = telemetry;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runShootingSpeed();
        startCountingTimer();
    }

    @Override
    public boolean isFinished() {
        // Keep calculating rolling averages of the shooter's RPM until numConsecutiveRequired in a row are within the specified RPM threshold
        // Implement sliding window algorithm to efficiently manage currentRPMSum
        currentRPMSum -= RPMReadings[currentIndex];
        RPMReadings[currentIndex] = shooter.getRPM();
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
            double error = shooter.getTargetRPM() - latestRPMRollingAverage;

            // Adjust numConsecutive accordingly
            if (error <= acceptableRPMThreshold)
                numConsecutive++;
            else
                numConsecutive = 0;

            if (numConsecutive >= numConsecutiveRequired)
                // The requirement has been met, it is ready
                return true;

            startCountingTimer();
        }


        telemetry.addData("Current Sum", currentRPMSum);
        telemetry.addData("Current RPM", shooter.getRPM());
        telemetry.addData("numConsecutive", numConsecutive);
        telemetry.addData("latest average", latestRPMRollingAverage);
        telemetry.addData("F value", shooter.getPIDFCoefficients().f);
        telemetry.update();

        return false;
    }

    // Schedule the next rolling average calculation
    private void startCountingTimer() {
        Timer startAverageTimer = new Timer();
        startAverageTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                countingReadings = true;
            }
        }, rollingAverageDelay);
    }
}
