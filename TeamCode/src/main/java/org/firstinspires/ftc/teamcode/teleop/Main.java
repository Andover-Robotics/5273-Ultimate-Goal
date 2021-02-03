package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Main TeleOp", group = "AA")
public class Main extends OpMode {

    private GamepadEx controller1, controller2;
    private MecanumDrive drive;
    private MotorEx intakeMotor, shooterMotor;

    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        shooterMotor = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        MotorEx motorFL = new MotorEx(hardwareMap, "motorFL", Motor.GoBILDA.RPM_312);
        MotorEx motorFR = new MotorEx(hardwareMap, "motorFR", Motor.GoBILDA.RPM_312);
        MotorEx motorBL = new MotorEx(hardwareMap, "motorBL", Motor.GoBILDA.RPM_312);
        MotorEx motorBR = new MotorEx(hardwareMap, "motorBR", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);
    }

    @Override
    public void loop() {
        telemetry.addLine("CONTROLS:");
        telemetry.addData("Triggers", "Shooter");
        telemetry.addData("Bumpers", "Intake");
        telemetry.addData("Controller 1 Left Stick", "Drive + Strafe");
        telemetry.addData("Controller 1 Right Stick", "Rotate");
        telemetry.update();


        if (controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02 || controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02) {
            shooterMotor.set(controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02 ? controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) : controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02 || controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02) {
            shooterMotor.set(-1 * (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02 ? controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) : controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        } else {
            shooterMotor.set(0);
        }

        if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeMotor.set(-1);
        } else if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER) || controller2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            intakeMotor.set(1);
        } else {
            intakeMotor.set(0);
        }

        drive.driveRobotCentric(-controller1.getLeftX(), -controller1.getLeftY(), controller1.getRightX());
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}
