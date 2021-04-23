package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RoadrunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicInteger;

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
        // Load in the F coefficient for the shooter
       /* try {
            Scanner scanner = new Scanner(new File("sdcard/FIRST/storedShooterFCoefficientAuto.txt"));
            double f = scanner.nextDouble();
            scanner.close();

            GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f = f;

            telemetry.addData("Found F", f);
            telemetry.update();
        } catch (FileNotFoundException e) {
            telemetry.addLine("Error loading shooter F coefficient! Using GlobalConfig's...");
            telemetry.update();
        }*/

        // Instantiate the MecanumDriveSubsystem, which uses a RoadrunnerMecanumDrive, which is bound
        // to the current config's drive train motor-naming scheme
        drive = new MecanumDriveSubsystem(new RoadrunnerMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(GlobalConfig.STARTING_POSITION);

        cartridge = new CartridgeSubsystem(hardwareMap, "cartridgeTilt", "cartridgeArm");
        cartridge.initCartridge();

        shooter = new ShooterSubsystem(hardwareMap, "shooter", GlobalConfig.AUTO_SHOOTER_RPM);

        wobbleGoalManipulator = new WobbleGoalManipulatorSubsystem(hardwareMap, "wobbleGoalTilt", "wobbleGoalClaw");
        wobbleGoalManipulator.lowerArm();
        wobbleGoalManipulator.openWide();


        sleep(3000);
        wobbleGoalManipulator.grip();
        sleep(1000);
        if (ringStackResult==RingStackDetector.RingStackResult.FOUR) {
            wobbleGoalManipulator.raiseArmFourRings();
        }
        else{
            wobbleGoalManipulator.raiseArm();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        ringStackDetector = new RingStackDetector(this, telemetry);

        sleep(500);

        AtomicInteger numProcessed = new AtomicInteger();

        while (numProcessed.get() < 25) {
            ringStackDetector.currentlyDetected()
                    .ifPresent((pair) -> {
                        numProcessed.getAndIncrement();
                        telemetry.addData("Status", "LOOKING FOR RINGS");
                        telemetry.addData("Detected Configuration", pair.first);
                        telemetry.addData("Detection Confidence", pair.second);
                        telemetry.addData("Processed", "%d frames", numProcessed.get());
                        telemetry.update();
                        RingStackDetector.RingStackResult thisResult = pair.first;
                        waitingRingStackConfidence = pair.second;

                        ringStackResult = thisResult;
                    });
        }

        sleep(1000);

        initialize();
        wobbleGoalManipulator.grip();

        Timer ringStackDetectorCloseTimer = new Timer();
        ringStackDetectorCloseTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                ringStackDetector.close();
            }
        }, 4000);

        while (!isStarted()) {
            telemetry.addData("Status", "READY");
            telemetry.addData("Detected Starter Stack", ringStackResult);
            telemetry.addData("Shooter F Coefficient (Use D-Pad to Configure)", GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f);
            telemetry.update();

            if (gamepad1.dpad_up)
                GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f += 0.01;
            else if (gamepad1.dpad_down)
                GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f -= 0.01;
            else if (gamepad1.dpad_left)
                GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f += 0.05;
            else if (gamepad1.dpad_right)
                GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f -= 0.05;

            // Rate-limit inputs
            sleep(250);
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

