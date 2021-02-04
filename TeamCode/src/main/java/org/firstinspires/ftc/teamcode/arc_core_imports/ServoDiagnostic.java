package org.firstinspires.ftc.teamcode.arc_core_imports;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.InputColumnResponder;
import org.firstinspires.ftc.teamcode.util.InputColumnResponderImpl;
import org.firstinspires.ftc.teamcode.util.Selector;

import java.util.Map;

@TeleOp(name = "Servo Diagnostic", group = "Test")
public class ServoDiagnostic extends OpMode {
    private static double POS_DELTA_LARGE = 0.05;
    private static double POS_DELTA_SMALL = 0.01;

    private Servo servo;
    private InputColumnResponder input = new InputColumnResponderImpl();
    private Selector servoSelector;

    @Override
    public void init() {
        servoSelector = new Selector(hardwareMap.servo.entrySet().stream().map(Map.Entry::getKey));
        input.register(() -> gamepad1.x, servoSelector::selectNext);
    }

    @Override
    public void init_loop() {
        input.update();
        telemetry.addData("Selected", servoSelector.selected());
        telemetry.addLine("Press X to select next");
    }

    @Override
    public void start() {
        servo = hardwareMap.servo.get(servoSelector.selected());
        input.clearRegistry();

        input.register(() -> gamepad1.dpad_up, () -> servo.setDirection(Servo.Direction.FORWARD))
                .register(() -> gamepad1.dpad_down, () -> servo.setDirection(Servo.Direction.REVERSE))
                .register(() -> gamepad1.y, () -> servo.setPosition(servo.getPosition() + POS_DELTA_LARGE))
                .register(() -> gamepad1.a, () -> servo.setPosition(servo.getPosition() - POS_DELTA_LARGE))
                .register(() -> gamepad1.x, () -> servo.setPosition(servo.getPosition() + POS_DELTA_SMALL))
                .register(() -> gamepad1.b, () -> servo.setPosition(servo.getPosition() - POS_DELTA_SMALL));
    }

    @Override
    public void loop() {
        telemetry.addLine("Controls")
                .addData("Dpad Up", "Sets direction to FORWARD")
                .addData("Dpad Down", "Sets direction to REVERSE")
                .addData("Y", "Increments position by %.3f", POS_DELTA_LARGE)
                .addData("A", "Decrements position by %.3f", POS_DELTA_LARGE)
                .addData("X", "Increments position by %.3f", POS_DELTA_SMALL)
                .addData("B", "Decrements position by %.3f", POS_DELTA_SMALL);

        telemetry.addLine("Servo Data")
                .addData("Direction", servo.getDirection().name())
                .addData("Position", "%.4f", servo.getPosition());

        input.update();
    }
}
