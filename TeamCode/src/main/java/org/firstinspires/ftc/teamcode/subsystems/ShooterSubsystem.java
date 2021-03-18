package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalConfig;

/*
public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx shooter;

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName) {
        this.shooter = new MotorEx(hardwareMap, shooterName);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setInverted(true);

        this.stop();
    }

    public void stop() {
        shooter.stopMotor();
    }

    public void setPower(double power) {
        shooter.set(Range.clip(power, 0, 1));
    }
}
*/

public class ShooterSubsystem extends SubsystemBase {
    private ShooterState shooterState;
    private final MotorEx shooter;
    private static FtcDashboard dashboard;

    private final static double ticksPerRevolution = 28.0;
    private final static double targetRPM = GlobalConfig.SHOOTER_RPM;
    private final static double targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;


    private enum ShooterState {
        OFF(0), SHOOT(targetTicksPerSec);
        public final double power;

        ShooterState(double power) {
            this.power = power;
        }
    }

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName) {
        this.shooter = new MotorEx(hardwareMap, shooterName);
        this.shooter.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.shooter.setInverted(true);

        this.shooterState = ShooterState.OFF;

        dashboard = FtcDashboard.getInstance();
        this.turnOff();
    }

    @Override
    public void periodic() {
        dashboard.getTelemetry().addData("shooter velocity", this.shooter.getVelocity());
    }

    public void runShootingSpeed() {
        shooterState = ShooterState.SHOOT;

        // Update motor by state
        shooter.motorEx.setVelocity(shooterState.power);
    }

    public void turnOff() {
        shooterState = ShooterState.OFF;

        // Update motor by state
        shooter.motorEx.setVelocity(shooterState.power);
    }

    public boolean isReadyToShoot() {
        return isFlywheelAtTargetVelocity(targetTicksPerSec);
    }

    private boolean isFlywheelAtTargetVelocity(double targetTicksPerSec) {
        // tolerance: rev/min * min/sec * ticks/rev = ticks / sec
        return Math.abs(targetTicksPerSec - shooter.getVelocity()) < 4.0 / 60 * 50;
    }
}


