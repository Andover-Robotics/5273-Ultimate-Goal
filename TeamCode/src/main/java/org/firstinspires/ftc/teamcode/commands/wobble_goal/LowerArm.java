package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class LowerArm extends CommandBase {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public LowerArm(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;
        addRequirements(wobbleGoalManipulator);
    }

    @Override
    public void initialize() {
        wobbleGoalManipulator.lowerArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
