package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
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
    private SimpleServo cartridgeTilt, cartridgeArm;
    private MecanumDrive drive;

    private ButtonReader gp2DPUpReader, gp2DPDownReader, gp2BReader;

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

        // Init ButtonReaders
        gp2DPUpReader = new ButtonReader(controller2, GamepadKeys.Button.DPAD_UP);
        gp2DPDownReader = new ButtonReader(controller2, GamepadKeys.Button.DPAD_DOWN);
        gp2BReader = new ButtonReader(controller2, GamepadKeys.Button.B);

        // Init + Reverse Motors and Servos
        intakeMotor = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intakeMotor.setInverted(true);

        shooterMotor = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        shooterMotor.setRunMode(Motor.RunMode.RawPower);
        shooterMotor.setInverted(true);

        cartridgeTilt = new SimpleServo(hardwareMap, "cartridgeTilt", 300, 0);
        cartridgeArm = new SimpleServo(hardwareMap, "cartridgeArm", 300, 0);

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
        cartridgeArm.turnToAngle(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_ANGLE);
        cartridgeTilt.turnToAngle(GlobalConfig.CARTRIDGE_LEVEL_ANGLE);
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
        telemetry.addLine("CONTROLLER 2:");
        telemetry.addData("Right Trigger", "Shooter");
        telemetry.addData("D-Pad Up", "Cartridge Shooter Position");
        telemetry.addData("D-Pad Down", "Cartridge Intake Position");
        telemetry.addData("B", "Push Ring Into Shooter");
        telemetry.update();

        // CONTROLLER 1
        manageIntake(controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        drive.driveRobotCentric(-controller1.getLeftX(), -controller1.getLeftY(), controller1.getRightX());

        // CONTROLLER 2
        manageShooter(controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        // CARTRIDGE MANAGEMENT
        if (gp2DPUpReader.wasJustPressed())
            cartridgeTilt.turnToAngle(GlobalConfig.CARTRIDGE_SHOOTER_ANGLE);
        else if (gp2DPDownReader.wasJustPressed())
            cartridgeTilt.turnToAngle(GlobalConfig.CARTRIDGE_INTAKE_ANGLE);

        // Only allow the cartridge arm to move when the cartridge is at the shooter angle and the arm is neutral
        if (gp2BReader.wasJustPressed() && Math.abs(cartridgeTilt.getAngle() - GlobalConfig.CARTRIDGE_SHOOTER_ANGLE) <= 0.25 && Math.abs(cartridgeArm.getAngle() - GlobalConfig.CARTRIDGE_ARM_NEUTRAL_ANGLE) <= 0.25) {
            // In case the above fails to catch a currently-executing cartridge arm command
            // If this if statement evaluates to true, we are not waiting on a thread related to the retraction of the cartridge arm
            if (retractCartridgeArmWhenReady == null || retractCartridgeArmWhenReady.isCancelled() || retractCartridgeArmWhenReady.isDone()) {
                cartridgeArm.turnToAngle(GlobalConfig.CARTRIDGE_ARM_PUSH_RING_ANGLE);
                retractCartridgeArmWhenReady = asyncExecutor.submit(retractArmWhenReady);
            }
        }
    }

    private void manageShooter(double triggerValue) {
        if (triggerValue > 0.05)
            if (triggerValue >= 0.95)
                shooterMotor.set(1);
            else
                shooterMotor.set(triggerValue);
        else
            shooterMotor.set(0);
    }

    private void manageIntake(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.05)
            if (rightTrigger >= 0.95)
                intakeMotor.set(1);
            else
                intakeMotor.set(rightTrigger);
        else if (leftTrigger >= 0.95)
            intakeMotor.set(-1);
        else
            intakeMotor.set(-leftTrigger);
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
                while (Math.abs(cartridgeArm.getAngle() - GlobalConfig.CARTRIDGE_ARM_NEUTRAL_ANGLE) > 0.25)
                    // Watch out for the cartridge moving again - don't want to break the arm or cause a jam
                    if (Math.abs(cartridgeTilt.getAngle() - GlobalConfig.CARTRIDGE_SHOOTER_ANGLE) > 0.5)
                        throw new InterruptedException(); // Uh oh, jump to that catch to turn back to neutral!


                // Give it some time for the ring to (hopefully) be grabbed by the flywheel
                wait(250);

                // Begin turning back to neutral
                cartridgeArm.turnToAngle(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_ANGLE);
            } catch (InterruptedException e) {
                // Turn back to neutral in case of error
                cartridgeArm.turnToAngle(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_ANGLE);
            }
        }
    };
}
