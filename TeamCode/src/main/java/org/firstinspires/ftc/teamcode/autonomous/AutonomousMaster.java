package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RoadrunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

public class AutonomousMaster extends CommandOpMode {

    protected MecanumDriveSubsystem drive;
    protected WobbleGoalManipulatorSubsystem wobbleGoalManipulator;
    protected CartridgeSubsystem cartridge;
    protected RingStackDetector ringStackDetector;
    protected RingStackDetector.RingStackResult ringStackResult;
    protected ShooterSubsystem shooter;
    protected double waitingRingStackConfidence;

    @Override
    public void initialize() {
        // Instantiate the MecanumDriveSubsystem, which uses a RoadrunnerMecanumDrive, which is bound
        // to the current config's drive train motor-naming scheme
        drive = new MecanumDriveSubsystem(new RoadrunnerMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(GlobalConfig.STARTING_POSITION);

        cartridge = new CartridgeSubsystem(hardwareMap, "cartridgeTilt", "cartridgeArm");
        cartridge.initCartridge();

        shooter = new ShooterSubsystem(hardwareMap, "shooter");

        wobbleGoalManipulator = new WobbleGoalManipulatorSubsystem(hardwareMap, "wobbleGoalTilt", "wobbleGoalClaw");
        wobbleGoalManipulator.lowerArm();
        wobbleGoalManipulator.release();
        sleep(3000);
        wobbleGoalManipulator.grip();
        sleep(1000);
        wobbleGoalManipulator.raiseArm();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        // TODO: Fix the RingStackDetector - it needs more time to get a good reading, so this needs to block the thread for a while
        // Possibly a loop and a counter that waits until a large number of outputs have been processed - a naive solution, but may work
        ringStackDetector = new RingStackDetector(this, telemetry);

        while (ringStackResult == null) {
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

        initialize();
        ringStackDetector.setFlashLight(true);
        wobbleGoalManipulator.grip();
        wobbleGoalManipulator.raiseArm();
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
                        RingStackDetector.RingStackResult thisRingStackResult = pair.first;
                        waitingRingStackConfidence = pair.second;

                        if (thisRingStackResult != null) {
                            ringStackResult = thisRingStackResult;
                        }
                    });
        }

        telemetry.addData("Status", "RUNNING");
        telemetry.update();
        // Run the scheduler (the same as CommandOpMode's code)
        while (!isStopRequested() && opModeIsActive()) {
            ringStackDetector.setFlashLight(false);
            run();
        }

        ringStackDetector.close();
        reset();
    }
}

