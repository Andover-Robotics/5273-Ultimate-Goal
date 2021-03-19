package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.RoadrunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@TeleOp(name = "Main TeleOp", group = "AA")
public class Main extends OpMode {

    private GamepadEx controller1, controller2;
    private MotorEx intakeMotor, shooterMotor;
    private SimpleServo cartridgeTilt, cartridgeArm, wobbleGoalTilt, wobbleGoalClaw;
    private MecanumDriveSubsystem drive;

    private Future<?> retractCartridgeArmWhenReady = null;
    private ExecutorService asyncExecutor;

    private double shooterPowerCap = GlobalConfig.SHOOTER_MAX_POWER;

    @Override
    public void init() {
        // Update Status
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        // Init asyncExecutor (for multithreading)
        asyncExecutor = Executors.newSingleThreadExecutor();

        // Init GamepadEx
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Init + Reverse Motors and Servos
        intakeMotor = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intakeMotor.setInverted(true);

        shooterMotor = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        shooterMotor.setRunMode(Motor.RunMode.RawPower);
        shooterMotor.setInverted(true);

        cartridgeTilt = new SimpleServo(hardwareMap, "cartridgeTilt", 300, 0);
        cartridgeArm = new SimpleServo(hardwareMap, "cartridgeArm", 300, 0);

        wobbleGoalTilt = new SimpleServo(hardwareMap, "wobbleGoalTilt", 300, 0);
        wobbleGoalClaw = new SimpleServo(hardwareMap, "wobbleGoalClaw", 300, 0);

        drive = new MecanumDriveSubsystem(new RoadrunnerMecanumDrive(hardwareMap), false);
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Set Initial Servo Positions / Angles
        setInitialPositions();

        // Update Status
        telemetry.addData("Status", "READY");
        telemetry.update();
    }

    private void setInitialPositions() {
        // Set servos to their proper default positions
        cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
        cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_LEVEL_POSITION);
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_TUCKED_POSITION);
        wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
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
        if (controller1.getButton(GamepadKeys.Button.Y))
            wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_DOWN_POSITION);
        else if (controller1.getButton(GamepadKeys.Button.X)) {
            wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
        } else if (controller1.getButton(GamepadKeys.Button.B)) {
            wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_GRAB_POSITION);
        } else if (controller1.getButton(GamepadKeys.Button.X))
            wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_OVER_WALL_POSITION);


        double speed = 0.30;
        if (controller1.getButton(GamepadKeys.Button.DPAD_UP)) {
            drive.drive(-speed, 0.0, 0.0);
        } else if (controller1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            drive.drive(speed, 0.0, 0.0);
        } else if (controller1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            drive.drive(0.0, -speed, 0.0);
        } else if (controller1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            drive.drive(0.0, speed, 0.0);
        } else {
            drive.drive(-controller1.getLeftY(), controller1.getLeftX(), controller1.getRightX());
        }
        drive.update();

        if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER))
            drive.drive(0.0, 0.0, Math.toRadians(20.0));
            //wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
        else if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            drive.drive(0.0, 0.0, Math.toRadians(-20.0));
        //wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_GRAB_POSITION);

        // CONTROLLER 2
        manageShooter(controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        if (controller2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
            shooterPowerCap = GlobalConfig.SHOOTER_LATER_RINGS_POWER;
        else if (controller2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
            shooterPowerCap = GlobalConfig.SHOOTER_MAX_POWER;

        // CARTRIDGE MANAGEMENT
        if (controller2.getButton(GamepadKeys.Button.DPAD_UP))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_SHOOTER_POSITION);
        else if (controller2.getButton(GamepadKeys.Button.DPAD_DOWN))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_INTAKE_POSITION);
        else if (controller2.getButton(GamepadKeys.Button.DPAD_LEFT) || controller2.getButton(GamepadKeys.Button.DPAD_RIGHT))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_LEVEL_POSITION);

        // Only allow the cartridge arm to move when the cartridge is at the shooter angle and the arm is neutral
        if (controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && Math.abs(cartridgeTilt.getPosition() - GlobalConfig.CARTRIDGE_SHOOTER_POSITION) <= 0.02 && Math.abs(cartridgeArm.getPosition() - GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION) <= 0.02) {
            // In case the above fails to catch a currently-executing cartridge arm command
            // If this if statement evaluates to true, we are not waiting on a thread related to the retraction of the cartridge arm
            if (retractCartridgeArmWhenReady == null || retractCartridgeArmWhenReady.isCancelled() || retractCartridgeArmWhenReady.isDone()) {
                cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_PUSH_RING_POSITION);
//                retractCartridgeArmWhenReady = asyncExecutor.submit(retractArmWhenReady);
            }
        }

        if (controller2.getButton(GamepadKeys.Button.LEFT_BUMPER))
            cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
    }

    private void manageShooter(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.05) {
            shooterMotor.set(Math.min(rightTrigger, shooterPowerCap));
        } else if (leftTrigger > 0.05) {
            shooterMotor.set(Range.clip((72 - Math.abs(drive.getPoseEstimate().getX())) * 0.01, 0, 1));
        } else {
            shooterMotor.stopMotor();
        }
    }

    private void manageIntake(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.05)
            intakeMotor.set(Math.min(rightTrigger, GlobalConfig.INTAKE_MAX_POWER));
        else if (leftTrigger > 0.05)
            intakeMotor.set(-1 * Math.min(leftTrigger, GlobalConfig.INTAKE_MAX_POWER));
        else
            intakeMotor.stopMotor();
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }

    Runnable retractArmWhenReady = new Runnable() {
        @Override
        public void run() {
            try {
                // Wait for the arm to have pushed the ring
                while (Math.abs(cartridgeArm.getPosition() - GlobalConfig.CARTRIDGE_ARM_PUSH_RING_POSITION) > 0.02) {
                    checkForInterrupt();

                    // Watch out for the cartridge moving again - don't want to break the arm or cause a jam
                    if (Math.abs(cartridgeTilt.getPosition() - GlobalConfig.CARTRIDGE_SHOOTER_POSITION) > 0.02)
                        throw new InterruptedException(); // Uh oh, jump to that catch to turn back to neutral!
                }


                // Give it some time for the ring to (hopefully) be grabbed by the flywheel
                wait(125);

                // Begin turning back to neutral
                cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
            } catch (InterruptedException e) {
                // Turn back to neutral in case of error
                cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
            }
        }
    };
}