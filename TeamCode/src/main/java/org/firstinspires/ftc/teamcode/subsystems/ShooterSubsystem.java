package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class ShooterSubsystem extends SubsystemBase {
    //    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 15, 14.35);
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(55, 0, 15, 13.9); // Gray flywheel
    public ShooterState shooterState;
    private ShooterType shooterType;
    private final DcMotorEx shooter;
    private static FtcDashboard dashboard;

    private final static double ticksPerRevolution = 28.0;
    private double targetRPM;
    private double targetTicksPerSec;


    public enum ShooterState {
        OFF, SHOOT
    }

    private enum ShooterType{
        HIGH_GOAL, POWER_SHOT
    }

    public ShooterSubsystem(HardwareMap hardwareMap, String shooterName, double targetRPM) {
        this.shooter = hardwareMap.get(DcMotorEx.class, shooterName);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12.0 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
        this.shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        this.shooterState = ShooterState.OFF;
        this.shooterType= ShooterType.HIGH_GOAL;

        this.targetRPM = targetRPM;
        this.targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;

        dashboard = FtcDashboard.getInstance();
        this.turnOff();
    }

    public void runShootingSpeed(int targetRPM){
        this.targetRPM=targetRPM;
        this.shooterType=ShooterType.HIGH_GOAL;
        this.targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;
        shooterState = ShooterState.SHOOT;

        // Update motor by state
        shooter.setVelocity(this.targetTicksPerSec);
    }


    public void runHighGoalShootingSpeed() {
        this.targetRPM=GlobalConfig.HIGH_GOAL_SHOOTER_RPM;
        this.shooterType=ShooterType.HIGH_GOAL;
        this.targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;
        shooterState = ShooterState.SHOOT;

        // Update motor by state
        shooter.setVelocity(this.targetTicksPerSec);
    }

    public void runPowerShotShootingSpeed() {
        this.targetRPM=GlobalConfig.POWER_SHOT_SHOOTER_RPM;
        this.shooterType=ShooterType.POWER_SHOT;
        this.targetTicksPerSec = ticksPerRevolution * targetRPM / 60.0;
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


