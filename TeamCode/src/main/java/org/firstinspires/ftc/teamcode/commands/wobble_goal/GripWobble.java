package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class GripWobble extends CommandBase {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public GripWobble(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;
        addRequirements(wobbleGoalManipulator);
    }

    @Override
    public void initialize() {
        wobbleGoalManipulator.grip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
