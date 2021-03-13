package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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
