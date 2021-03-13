package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class RaiseArm extends CommandBase {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public RaiseArm(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;
        addRequirements(wobbleGoalManipulator);
    }

    @Override
    public void initialize() {
        wobbleGoalManipulator.raiseArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
