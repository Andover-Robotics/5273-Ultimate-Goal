package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Intake + Shooter Test TeleOp", group = "Test")
public class IntakeShooterTest extends OpMode {

    private GamepadEx controller1, controller2;
    private Motor motorFR, motorFL, motorBR, motorBL;
    private MotorEx intakeMotor, shooterMotor;

    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        shooterMotor = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.RPM_1150);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        telemetry.addLine("CONTROLS:");
        telemetry.addData("Triggers", "Shooter");
        telemetry.addData("Bumpers", "Intake");
        telemetry.update();


        if (controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02 || controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02) {
            shooterMotor.set(controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.02 ? controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) : controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02 || controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02) {
            shooterMotor.set(-1 * (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.02 ? controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) : controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        }

        if (controller1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || controller2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            intakeMotor.set(1);
        } else if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER) || controller2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            intakeMotor.set(-1);
        }
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}
