package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class ReleaseWobble extends CommandBase {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public ReleaseWobble(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;
        addRequirements(wobbleGoalManipulator);
    }

    @Override
    public void initialize() {
        wobbleGoalManipulator.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
