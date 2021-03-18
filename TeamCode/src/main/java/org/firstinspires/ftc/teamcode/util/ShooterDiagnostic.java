package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;

import java.io.FileNotFoundException;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@TeleOp(
        name = "Shooter Diagnostic",
        group = "Experimental"
)
public final class ShooterDiagnostic extends OpMode {
    private DcMotorEx flywheel;
    private CartridgeSubsystem cartridge;
    private VoltageSensor volts;

    private FtcDashboard dash;
    private GamepadEx pad;
    private StreamingDataWriter writer;

    private int shootRpm = 4700; // Theoretical, calculated value with approximations
    private int feedGapDuration = 750;
    private final int feedDuration = 400;

    // F - feedDuration - B - feedGapDuration - repeat
    private Double feederStartTime = null;

    public void init() {
        this.flywheel = this.hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(10.0, 3.0, 0.3, 0.0);

        cartridge = new CartridgeSubsystem(hardwareMap, "cartridgeTilt", "cartridgeArm");
        cartridge.resetArm();
        cartridge.raiseCartridge();

        volts = this.hardwareMap.voltageSensor.iterator().next();
        this.dash = FtcDashboard.getInstance();
        pad = new GamepadEx(gamepad1);
        try {
            writer = new StreamingDataWriter(20, () -> {
                PIDFCoefficients pid = flywheel.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
                return new double[] {
                        getRuntime(),
                        shootRpm,
                        feedDuration,
                        feedGapDuration,
                        flywheel.getVelocity() * 60.0 / 28.0,
                        volts.getVoltage(),
                        pid.p,
                        pid.i,
                        pid.d
                };
            }, "/sdcard/FIRST/shooter-diagnostic" + (int)Math.floor(Math.random() * 1000) + ".csv");
            writer.writeLine("time,target,feed,gap,velocity,voltage,p,i,d");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            stop();
        }
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
    }

    public void start() {
        writer.startStreaming();
    }

    public void loop() {
        if (pad.wasJustPressed(DPAD_UP)) {
            shootRpm += 100;
        } else if (pad.wasJustPressed(DPAD_DOWN)) {
            shootRpm -= 100;
        } else if (pad.wasJustPressed(DPAD_LEFT)) {
            feedGapDuration += 100;
        } else if (pad.wasJustPressed(DPAD_RIGHT) && feedGapDuration > 100) {
            feedGapDuration -= 100;
        }
        applyShootRpm();

        if (pad.wasJustPressed(Y)) {
            feederStartTime = getRuntime();
        }
        if (feederStartTime != null) {
            feederLoop();
        }

        telemetry.addData("flywheel rpm", flywheel.getVelocity() * 60.0 / 28.0);
        telemetry.addData("flywheel target rpm", shootRpm);
        telemetry.addData("feeder duration", feedGapDuration);
        adjustShootPID();

        pad.readButtons();
    }

    public void stop() {
        writer.stopStreaming();
    }

    public final void applyShootRpm() {
        flywheel.setVelocity(28.0 * this.shootRpm / 60.0);
    }

    public final void adjustShootPID() {
        PIDFCoefficients current = flywheel.getPIDFCoefficients(RunMode.RUN_USING_ENCODER);
        telemetry.addData("flywheel PIDF", "%.2f, %.2f, %.2f, %.2f", current.p, current.i, current.d, current.f);
        if (pad.wasJustPressed(LEFT_STICK_BUTTON)) {
            current.p += 1;
        } else if (pad.wasJustPressed(RIGHT_STICK_BUTTON)) {
            current.i += 0.1;
        } else if (pad.wasJustPressed(LEFT_BUMPER)) {
            current.p -= 1;
        } else if (pad.wasJustPressed(RIGHT_BUMPER)) {
            current.i -= 0.1;
        }
        flywheel.setVelocityPIDFCoefficients(current.p, current.i, current.d, current.f);
    }

    private double[] incidentRPMs = {0, 0, 0};

    public final void feederLoop() {
        double dt = (getRuntime() - feederStartTime) * 1000;
        if (dt / (feedGapDuration + feedDuration) > 3) {
            feederStartTime = null;
        } else {
            double iterDt = dt % (feedGapDuration + feedDuration);

            if(iterDt >= feedDuration)
                cartridge.resetArm();
            else
                cartridge.pushArm();
        }
    }
}
