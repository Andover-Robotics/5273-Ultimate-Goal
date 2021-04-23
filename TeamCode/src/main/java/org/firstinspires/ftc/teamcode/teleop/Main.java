package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.RoadrunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@TeleOp(name = "Main TeleOp", group = "AA")
public class Main extends OpMode {

    private FtcDashboard dash;
    private GamepadEx controller1, controller2;
    private MotorEx intakeMotor;
    private MecanumDriveSubsystem drive;
    private ShooterSubsystem shooter;
    private CartridgeSubsystem cartridge;
    private WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    private Future<?> retractCartridgeArmWhenReady = null;
    private ExecutorService asyncExecutor;

    private double shooterPowerCap = GlobalConfig.SHOOTER_MAX_POWER;

    private enum WobbleGoalArmState {
        TUCKED, LOWERED, CARRYING, RAISED
    }

    private enum ControlMode {
        DRIVER_CONTROL, AUTOMATIC_CONTROL
    }

    private WobbleGoalArmState wobbleGoalArmState;
    private ControlMode currentControlMode;

    // Adjust according to preference and what needs to be debugged
    private final boolean SHOULD_LOG_SHOOTER_RPM = false;

    @Override
    public void init() {
        // Update Status
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        /*try {
            Scanner scanner = new Scanner(new File("sdcard/FIRST/storedShooterFCoefficientTeleOp.txt"));
            double f = scanner.nextDouble();
            scanner.close();

            GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f = f;

            telemetry.addData("Found F", f);
            telemetry.update();
        } catch (FileNotFoundException e) {
            telemetry.addLine("Error loading shooter F coefficient! Using GlobalConfig's...");
            telemetry.update();
        }*/

        // Init asyncExecutor (for multithreading)
        asyncExecutor = Executors.newSingleThreadExecutor();

        // Init GamepadEx
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Init + Reverse Motors and Servos
        intakeMotor = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intakeMotor.setInverted(true);

        shooter = new ShooterSubsystem(hardwareMap, "shooter", GlobalConfig.HIGH_GOAL_SHOOTER_RPM);

        cartridge = new CartridgeSubsystem(hardwareMap, "cartridgeTilt", "cartridgeArm");

        wobbleGoalManipulator = new WobbleGoalManipulatorSubsystem(hardwareMap, "wobbleGoalTilt", "wobbleGoalClaw");

        drive = new MecanumDriveSubsystem(new RoadrunnerMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(PoseStorage.currentPose);
        //drive.setPoseEstimate(new Pose2d(GlobalConfig.PARKING_POSITION.getX(), GlobalConfig.PARKING_POSITION.getY(), PoseStorage.currentPose.getHeading()));

        dash = FtcDashboard.getInstance();

        // Set Initial Servo Positions / Angles
        setInitialPositions();

        // Default to driver-controlled mode
        currentControlMode = ControlMode.DRIVER_CONTROL;

        // Update Status
        telemetry.addData("Status", "READY");
        telemetry.update();
    }

    private void setInitialPositions() {
        // Set servos to their proper default positions
        cartridge.lowerCartridge();
        cartridge.resetArm();
        wobbleGoalManipulator.openWide();

        wobbleGoalArmState = WobbleGoalArmState.TUCKED;
        wobbleGoalManipulator.tuckArm();
    }


    @Override
    public void loop() {
        // Log Controls
        telemetry.addData("CURRENT MODE:", currentControlMode.name());
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("CONTROLLER 1:");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick", "Rotate");
        telemetry.addData("Right Trigger", "Intake (Pressure-sensitive)");
        telemetry.addData("Left Trigger", "Outtake (Reverse Intake) (Pressure-sensitive)");
        telemetry.addData("D-Pad Up", "Drive Forward (SLOW MODE)");
        telemetry.addData("D-Pad Down", "Drive Backward (SLOW MODE)");
        telemetry.addData("D-Pad L/R", "Strafe Left/Right (SLOW MODE)");
        telemetry.addData("L/R Bumpers", "Turn Left/Right (SLOW MODE)");
        telemetry.addData("X", "Release wobble goal");
        telemetry.addData("B", "Grip wobble goal. Lowers arm at the same time if the arm was already CARRYING.");
        telemetry.addData("A", "RAISE wobble goal arm");
        telemetry.addData("Y", "CARRY wobble goal (slightly raise off ground)");
        telemetry.addData("Back", "TUCK wobble goal arm (raise over wall)");
        telemetry.addLine("CONTROLLER 2:");
        telemetry.addData("D-Pad Up", "Cartridge Shooter Position");
        telemetry.addData("D-Pad Down", "Cartridge Intake Position");
        telemetry.addData("D-Pad L/R", "Cartridge Level Position");
        telemetry.addData("Y", "Run shooter for HIGH GOAL");
        telemetry.addData("A", "Run shooter for POWER SHOTS");
        telemetry.addData("B", "Run shooter for HIGH GOAL FROM WOBBLE GOAL DELIVERY");
        telemetry.addData("X", "Stop shooter");
        telemetry.addData("Right trigger", "Push ring into shooter");
        telemetry.addData("Left trigger", "Retract cartridge arm");
        telemetry.addData("Left bumper", "SPLINE to shooting position");
        telemetry.addData("Right bumper", "LINE to shooting position");
        telemetry.addData("Back (while in driver control)", "Reset pose estimate to nearest corner");
        telemetry.addData("Back (while following trajectory)", "Cancel trajectory following, cede control to drivers");
        telemetry.update();

        // UPDATE drive - it is VERY important that this is outside of the switch statement
        drive.update();

        // CONTROLS UNAFFECTED BY CURRENT ControlMode:
        // CONTROLLER 1
        manageIntake(controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        // WOBBLE GOAL MANIPULATOR
        if (controller1.wasJustPressed(GamepadKeys.Button.X))
            wobbleGoalManipulator.openWide();
        else if (controller1.wasJustPressed(GamepadKeys.Button.B)) {
            wobbleGoalManipulator.grip();

            // Lower and grip at the same time if from carrying state
            if (wobbleGoalArmState == WobbleGoalArmState.CARRYING) {
                wobbleGoalManipulator.lowerArm();
                wobbleGoalArmState = WobbleGoalArmState.LOWERED;
            }
        } else if (controller1.wasJustPressed(GamepadKeys.Button.A)) {
            wobbleGoalManipulator.raiseOverWall();
            wobbleGoalArmState = WobbleGoalArmState.RAISED;
        } else if (controller1.wasJustPressed(GamepadKeys.Button.BACK)) {
            wobbleGoalManipulator.tuckArm();
            wobbleGoalArmState = WobbleGoalArmState.TUCKED;
        } else if (controller1.wasJustPressed(GamepadKeys.Button.Y)) {
            wobbleGoalManipulator.raiseArm();
            wobbleGoalArmState = WobbleGoalArmState.CARRYING;
        }

        // CONTROLLER 2
        // SHOOTER FLYWHEEL SPEED
        if (controller2.wasJustPressed(GamepadKeys.Button.Y))
            shooter.runHighGoalShootingSpeed();
        else if (controller2.wasJustPressed(GamepadKeys.Button.A))
            shooter.runPowerShotShootingSpeed();
        else if (controller2.wasJustPressed(GamepadKeys.Button.X))
            shooter.turnOff();
        else if (controller2.wasJustPressed(GamepadKeys.Button.B))
            // Speed for shooting extra rings in end game after delivering a wobble goal
            shooter.runShootingSpeed(GlobalConfig.HIGH_GOAL_SHOOTER_RPM - 250);

        // Construct and send shooter TelemetryPacket if needed
        if (SHOULD_LOG_SHOOTER_RPM) {
            TelemetryPacket shooterTelemetry = new TelemetryPacket();
            shooterTelemetry.put("RPM", shooter.getRPM());
            shooterTelemetry.put("Target RPM", shooter.getTargetRPM());
            dash.sendTelemetryPacket(shooterTelemetry);
        }

        // CARTRIDGE MANAGEMENT
        if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            cartridge.raiseCartridge();
        else if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            cartridge.lowerCartridge();
        else if (controller2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || controller2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            cartridge.levelCartridge();

        if (controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
            cartridge.pushArm();
        else if (controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
            cartridge.resetArm();

        // CONTROLS DETERMINED BY CURRENT ControlMode:
        switch (currentControlMode) {
            case AUTOMATIC_CONTROL:
                // AUTOMATIC CONTROL MODE (TRAJECTORY FOLLOWING)
                // We basically do nothing because we're waiting for the trajectory to finish
                // If the back button is pressed, we cancel the automatic following
                if (controller2.wasJustPressed(GamepadKeys.Button.BACK)) {
                    drive.stop();
                    currentControlMode = ControlMode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentControlMode = ControlMode.DRIVER_CONTROL;
                }
                break;
            case DRIVER_CONTROL:
                // DRIVER CONTROL MODE
                // Handle driving, turning, and slow modes for driving/strafing/turning
                double moveSpeed = 0.4, turnSpeedDegrees = 15;
                if (controller1.getButton(GamepadKeys.Button.DPAD_UP))
                    drive.drive(-moveSpeed, 0.0, 0.0);
                else if (controller1.getButton(GamepadKeys.Button.DPAD_DOWN))
                    drive.drive(moveSpeed, 0.0, 0.0);
                else if (controller1.getButton(GamepadKeys.Button.DPAD_LEFT))
                    drive.drive(0.0, -moveSpeed, 0.0);
                else if (controller1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                    drive.drive(0.0, moveSpeed, 0.0);
                else if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                    drive.drive(0.0, 0.0, -Math.toRadians(turnSpeedDegrees));
                else if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                    drive.drive(0.0, 0.0, Math.toRadians(turnSpeedDegrees));
                else
                    drive.drive(-controller1.getLeftY(), controller1.getLeftX(), controller1.getRightX());

                // Get current pose estimate
                Pose2d poseEstimate = drive.getPoseEstimate();

                // Handle the commands to begin following a trajectory
                if (controller2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    // Build and begin following a SPLINETO command to get to the shooting position

                    // First, we need to determine what endTangent to use, based on poseEstimate
                    // This is not super important but will help make a more efficient path

                    // If we are above the shooting position, approach it from the tower goal side
                    // If not, approach from the starting side
                    double endTangent = (poseEstimate.getX() > GlobalConfig.RING_SHOOTING_POSITION.getX()) ? 0 : Math.toRadians(180);
                    // Update the startHeading in a similar way
                    double startHeading = (poseEstimate.getX() > GlobalConfig.RING_SHOOTING_POSITION.getX()) ? Math.toRadians(180) : 0;

                    // Build the trajectory
                    Trajectory moveToShoot = drive.trajectoryBuilder(poseEstimate, startHeading)
                            .splineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION, endTangent)
                            .build();

                    // Begin async following and switch control mode accordingly
                    drive.followTrajectory(moveToShoot);

                    currentControlMode = ControlMode.AUTOMATIC_CONTROL;
                } else if (controller2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    // Build and begin following a LINETO command to get to the shooting position
                    Trajectory moveToShoot = drive.trajectoryBuilder(poseEstimate)
                            .lineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION)
                            .build();

                    // Begin async following and switch control mode accordingly
                    drive.followTrajectory(moveToShoot);

                    currentControlMode = ControlMode.AUTOMATIC_CONTROL;
                }

                // Use controller 2's back button to reset the pose estimate
                if (controller2.wasJustPressed(GamepadKeys.Button.BACK)) {
                    double estimatedX = poseEstimate.getX();
                    double estimatedY = poseEstimate.getY();
                    // Adjust the pose estimate according to the quadrant (heading is maintained)
                    if (estimatedX > 0 && estimatedY > 0)
                        drive.setPoseEstimate(new Pose2d(72, 72, drive.getPoseEstimate().getHeading()));
                    else if (estimatedX > 0 && estimatedY < 0)
                        drive.setPoseEstimate(new Pose2d(72, -72, drive.getPoseEstimate().getHeading()));
                    else if (estimatedX < 0 && estimatedY > 0)
                        drive.setPoseEstimate(new Pose2d(-24, 72, drive.getPoseEstimate().getHeading()));
                    else
                        drive.setPoseEstimate(new Pose2d(-24, -72, drive.getPoseEstimate().getHeading()));
                }
                break;
        }

        // Update button readings so we can use wasJustPressed()
        controller1.readButtons();
        controller2.readButtons();
    }

    private void manageIntake(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.05)
            intakeMotor.set(GlobalConfig.INTAKE_MAX_POWER);
        else if (leftTrigger > 0.05)
            intakeMotor.set(-1 * GlobalConfig.INTAKE_MAX_POWER);

        else
            intakeMotor.stopMotor();
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}