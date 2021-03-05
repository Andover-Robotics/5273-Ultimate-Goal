package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@TeleOp(name = "Main TeleOp", group = "AA")
public class Main extends OpMode {

    private GamepadEx controller1, controller2;
    private MotorEx intakeMotor, shooterMotor;
    private SimpleServo cartridgeTilt, cartridgeArm, wobbleGoalTilt, wobbleGoalClaw;
    private MecanumDrive drive;

    private Future<?> retractCartridgeArmWhenReady = null;
    private ExecutorService asyncExecutor;

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

        // Init MecanumDrive
        MotorEx motorFL = new MotorEx(hardwareMap, "motorFL", Motor.GoBILDA.RPM_312);
        MotorEx motorFR = new MotorEx(hardwareMap, "motorFR", Motor.GoBILDA.RPM_312);
        MotorEx motorBL = new MotorEx(hardwareMap, "motorBL", Motor.GoBILDA.RPM_312);
        MotorEx motorBR = new MotorEx(hardwareMap, "motorBR", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);

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
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_UP_POSITION);
        wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
    }


    @Override
    public void loop() {
        // Log Controls
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("CONTROLLER 1:");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick", "Rotate");
        telemetry.addData("Left Trigger", "Outtake (Reverse Intake)");
        telemetry.addData("Right Trigger", "Intake");
        telemetry.addData("Y", "Wobble Arm Up");
        telemetry.addData("X", "Wobble Arm Middle");
        telemetry.addData("A", "Wobble Arm Down");
        telemetry.addData("Left Bumper", "Wobble Claw Release");
        telemetry.addData("Right Bumper", "Wobble Claw Grab");
        telemetry.addLine("CONTROLLER 2:");
        telemetry.addData("Right Trigger", "Shooter");
        telemetry.addData("D-Pad Up", "Cartridge Shooter Position");
        telemetry.addData("D-Pad Down", "Cartridge Intake Position");
        telemetry.addData("D-Pad L/R", "Cartridge Level Position");
        telemetry.addData("B", "Push Ring Into Shooter");
        telemetry.update();

        // CONTROLLER 1
        manageIntake(controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        drive.driveRobotCentric(-controller1.getLeftX(), -controller1.getLeftY(), controller1.getRightX());

        // WOBBLE GOAL ARM
        if (controller1.getButton(GamepadKeys.Button.Y))
            wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_UP_POSITION);
        else if (controller1.getButton(GamepadKeys.Button.A))
            wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_DOWN_POSITION);
        else if (controller1.getButton(GamepadKeys.Button.X))
            wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_NEUTRAL_POSITION);

        if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER))
            wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
        else if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_GRAB_POSITION);

        // CONTROLLER 2
        manageShooter(controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        // CARTRIDGE MANAGEMENT
        if (controller2.getButton(GamepadKeys.Button.DPAD_UP))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_SHOOTER_POSITION);
        else if (controller2.getButton(GamepadKeys.Button.DPAD_DOWN))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_INTAKE_POSITION);
        else if (controller2.getButton(GamepadKeys.Button.DPAD_LEFT) || controller2.getButton(GamepadKeys.Button.DPAD_RIGHT))
            cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_LEVEL_POSITION);

        // Only allow the cartridge arm to move when the cartridge is at the shooter angle and the arm is neutral
        if (controller2.getButton(GamepadKeys.Button.B) && Math.abs(cartridgeTilt.getPosition() - GlobalConfig.CARTRIDGE_SHOOTER_POSITION) <= 0.02 && Math.abs(cartridgeArm.getPosition() - GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION) <= 0.02) {
            // In case the above fails to catch a currently-executing cartridge arm command
            // If this if statement evaluates to true, we are not waiting on a thread related to the retraction of the cartridge arm
            if (retractCartridgeArmWhenReady == null || retractCartridgeArmWhenReady.isCancelled() || retractCartridgeArmWhenReady.isDone()) {
                cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_PUSH_RING_POSITION);
//                retractCartridgeArmWhenReady = asyncExecutor.submit(retractArmWhenReady);
            }
        }

        if (controller2.getButton(GamepadKeys.Button.X))
            cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
    }

    private void manageShooter(double triggerValue) {
        if (triggerValue > 0.05)
            shooterMotor.set(Math.min(triggerValue, GlobalConfig.SHOOTER_MAX_POWER));
        else
            shooterMotor.stopMotor();
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