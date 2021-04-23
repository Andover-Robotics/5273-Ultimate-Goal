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

    private WobbleGoalArmState wobbleGoalArmState;

    @Override
    public void init() {
        // Update Status
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        try {
            Scanner scanner = new Scanner(new File("sdcard/FIRST/storedShooterFCoefficientTeleOp.txt"));
            double f = scanner.nextDouble();
            scanner.close();

            GlobalConfig.SHOOTER_PIDF_COEFFICIENTS.f = f;

            telemetry.addData("Found F", f);
            telemetry.update();
        } catch (FileNotFoundException e) {
            telemetry.addLine("Error loading shooter F coefficient! Using GlobalConfig's...");
            telemetry.update();
        }

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
        telemetry.addData("shooterPowerCap", shooterPowerCap);
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("CONTROLLER 1:");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick", "Rotate");
        telemetry.addData("Left Trigger", "Outtake (Reverse Intake)");
        telemetry.addData("Right Trigger", "Intake");
        telemetry.addData("D-Pad Up", "Drive Forward");
        telemetry.addData("D-Pad Down", "Drive in Reverse");
        telemetry.addData("D-Pad L/R", "Strafe Left/Right");
        telemetry.addData("Y", "Wobble Arm Up");
        telemetry.addData("X", "Wobble Arm Middle");
        telemetry.addData("A", "Wobble Arm Down");
        telemetry.addData("Left Bumper", "Wobble Claw Release");
        telemetry.addData("Right Bumper", "Wobble Claw Grab");
        telemetry.addLine("CONTROLLER 2:");
        telemetry.addData("Right Trigger", "Shooter");
        telemetry.addData("Left Trigger", "Adjusted Shooter");
        telemetry.addData("D-Pad Up", "Cartridge Shooter Position");
        telemetry.addData("D-Pad Down", "Cartridge Intake Position");
        telemetry.addData("D-Pad L/R", "Cartridge Level Position");
        telemetry.addData("B", "Push Ring Into Shooter");
        telemetry.update();

        // CONTROLLER 1
        manageIntake(controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        // WOBBLE GOAL ARM
        if (controller1.getButton(GamepadKeys.Button.X)) {
            wobbleGoalManipulator.openWide();
        } else if (controller1.getButton(GamepadKeys.Button.B)) {
            wobbleGoalManipulator.grip();
        }
        else if(controller1.getButton(GamepadKeys.Button.A)){
            wobbleGoalManipulator.raiseArm();
            wobbleGoalArmState=WobbleGoalArmState.RAISED;
        }
         else if (controller1.getButton(GamepadKeys.Button.BACK)) {
            wobbleGoalManipulator.tuckArm();
            wobbleGoalArmState = WobbleGoalArmState.TUCKED;
        }


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

        drive.update();


        // CONTROLLER 2
//        manageShooter(controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
//        if (controller2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
//            shooterPowerCap = GlobalConfig.SHOOTER_LATER_RINGS_POWER;
//        else if (controller2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
//            shooterPowerCap = GlobalConfig.SHOOTER_MAX_POWER;

        if (controller2.getButton(GamepadKeys.Button.Y)) {
            shooter.runHighGoalShootingSpeed();
        }
        else if (controller2.getButton(GamepadKeys.Button.A)) {
            shooter.runPowerShotShootingSpeed();
        }
        else if( controller2.getButton(GamepadKeys.Button.X)){
            shooter.turnOff();
        }

        /*
        if (controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            drive.turn(drive.getPoseEstimate().getHeading()-Math.toRadians(180.0));
        }
        else if(controller2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).splineToLinearHeading(GlobalConfig.COLLECT_OTHER_WOBBLE, Math.toRadians(0.0)).build());
        }
        */

        /*if (controller2.getButton(GamepadKeys.Button.B)){
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            if (x>0 && y>0){
                drive.setPoseEstimate(new Pose2d(72, 72, drive.getPoseEstimate().getHeading()));
            }
            else if(x>0 && y<0){
                drive.setPoseEstimate(new Pose2d(72, -72, drive.getPoseEstimate().getHeading()));
            }
            else if(x<0 && y>0){
                drive.setPoseEstimate(new Pose2d(-72, 72, drive.getPoseEstimate().getHeading()));
            }
            else{
                drive.setPoseEstimate(new Pose2d(-72, -72, drive.getPoseEstimate().getHeading()));
            }
        }
        */

        if (controller2.getButton(GamepadKeys.Button.B)){
            shooter.runShootingSpeed(GlobalConfig.HIGH_GOAL_SHOOTER_RPM-250);
        }

        // CARTRIDGE MANAGEMENT
        if (controller2.getButton(GamepadKeys.Button.DPAD_UP))
            cartridge.raiseCartridge();
        else if (controller2.getButton(GamepadKeys.Button.DPAD_DOWN))
            cartridge.lowerCartridge();
        else if (controller2.getButton(GamepadKeys.Button.DPAD_LEFT) || controller2.getButton(GamepadKeys.Button.DPAD_RIGHT))
            cartridge.levelCartridge();

        if (controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)> 0.05)
            cartridge.pushArm();
        else if (controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.05)
            cartridge.resetArm();

        if (controller1.wasJustPressed(GamepadKeys.Button.Y)) {
            if (wobbleGoalArmState == WobbleGoalArmState.LOWERED) {
                wobbleGoalManipulator.raiseArm();
                wobbleGoalArmState = WobbleGoalArmState.CARRYING;
            } else {
                wobbleGoalManipulator.raiseOverWall();
                wobbleGoalArmState = WobbleGoalArmState.RAISED;
            }
        } else if (controller1.getButton(GamepadKeys.Button.B)) {
            wobbleGoalManipulator.lowerArm();
            wobbleGoalArmState = WobbleGoalArmState.LOWERED;
        }

        // Only allow the cartridge arm to move when the cartridge is at the shooter angle and the arm is neutral
        /*if (controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && Math.abs(cartridgeTilt.getPosition() - GlobalConfig.CARTRIDGE_SHOOTER_POSITION) <= 0.02 && Math.abs(cartridgeArm.getPosition() - GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION) <= 0.02) {
            // In case the above fails to catch a currently-executing cartridge arm command
            // If this if statement evaluates to true, we are not waiting on a thread related to the retraction of the cartridge arm
            if (retractCartridgeArmWhenReady == null || retractCartridgeArmWhenReady.isCancelled() || retractCartridgeArmWhenReady.isDone()) {
                cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_PUSH_RING_POSITION);
//                retractCartridgeArmWhenReady = asyncExecutor.submit(retractArmWhenReady);
            }
        }

        if (controller2.getButton(GamepadKeys.Button.LEFT_BUMPER))
            cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);*/

        TelemetryPacket shooterTelemetry = new TelemetryPacket();
        shooterTelemetry.put("RPM", shooter.getRPM());
        shooterTelemetry.put("Target RPM", shooter.getTargetRPM());
        dash.sendTelemetryPacket(shooterTelemetry);

        controller1.readButtons();
        controller2.readButtons();
    }

    private void manageIntake(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.05)
            intakeMotor.set(GlobalConfig.INTAKE_MAX_POWER);
        else if (leftTrigger > 0.05)
            intakeMotor.set(-1 *GlobalConfig.INTAKE_MAX_POWER);

        else
            intakeMotor.stopMotor();
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}