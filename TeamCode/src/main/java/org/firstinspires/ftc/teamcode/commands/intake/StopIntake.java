package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class StopIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public StopIntake(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem=intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
