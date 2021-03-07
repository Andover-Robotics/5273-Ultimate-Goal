package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.drive.RoadrunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

public class AutonomousMaster extends CommandOpMode {

    protected MecanumDriveSubsystem drive;
    protected RingStackDetector ringStackDetector;
    protected RingStackDetector.RingStackResult ringStackResult;
    protected double waitingRingStackConfidence;

    @Override
    public void initialize() {
        // Instantiate the MecanumDriveSubsystem, which uses a RoadrunnerMecanumDrive, which is bound
        // to the current config's drive train motor-naming scheme
        drive = new MecanumDriveSubsystem(new RoadrunnerMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        ringStackDetector = new RingStackDetector(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();
        initialize();

//        waitForStart(); -- Replaced with RingStackDetector waiting
        while (!isStarted()) {
            if (isStopRequested()) return;
            // keep getting results from the pipeline
            ringStackDetector.currentlyDetected()
                    .ifPresent((pair) -> {
                        telemetry.addData("Status", "READY");
                        telemetry.addData("Detected Configuration", pair.first);
                        telemetry.addData("Detection Confidence", pair.second);
                        telemetry.update();
                        ringStackResult = pair.first;
                        waitingRingStackConfidence = pair.second;
                    });
        }

        telemetry.addData("Status", "RUNNING");
        telemetry.update();
        // Run the scheduler (the same as CommandOpMode's code)
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}

