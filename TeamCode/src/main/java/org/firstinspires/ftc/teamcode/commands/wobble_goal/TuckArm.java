package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class TuckArm extends CommandBase {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public TuckArm(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;
        addRequirements(wobbleGoalManipulator);
    }

    @Override
    public void initialize() {
        wobbleGoalManipulator.tuckArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
