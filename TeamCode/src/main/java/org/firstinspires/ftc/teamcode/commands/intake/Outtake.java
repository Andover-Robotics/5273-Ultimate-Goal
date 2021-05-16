package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Outtake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public Outtake( IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.reverseIntake();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
