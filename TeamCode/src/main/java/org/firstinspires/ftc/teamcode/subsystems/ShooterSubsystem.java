package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterState shooterState;
    private final DcMotorEx shooter;
    private static FtcDashboard dashboard;

    private final static double ticksPerRevolution = 28.0;
    private double targetRPM;
    private double targetTicksPerSec;


    private enum ShooterState {
        OFF, SHOOT
    }

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName, double targetRPM) {
        this.shooter = hardwareMap.get(DcMotorEx.class, shooterName);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, GlobalConfig.SHOOTER_PIDF_COEFFICIENTS);
        this.shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        this.shooterState = ShooterState.OFF;

        this.targetRPM = targetRPM;
        this.targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;

        dashboard = FtcDashboard.getInstance();
        this.turnOff();
    }

    public void runShootingSpeed() {
        shooterState = ShooterState.SHOOT;

        // Update motor by state
        shooter.setVelocity(this.targetTicksPerSec);
    }

    public void turnOff() {
        shooterState = ShooterState.OFF;

        // Update motor by state
        shooter.setVelocity(0);
    }

    public void decrementRPM() {
        this.targetRPM -= 37.5;
        this.targetTicksPerSec = ticksPerRevolution * this.targetRPM / 60.0;

        runShootingSpeed();
    }

    public double getRPM() {
        return shooter.getVelocity() * 60.0 / ticksPerRevolution;
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}


