package org.firstinspires.ftc.teamcode.commands.shooting;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class StartShooter extends CommandBase {
    private ShooterSubsystem shooter;

    public StartShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(GlobalConfig.SHOOTER_MAX_POWER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
