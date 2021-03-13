package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class DropWobbleGoal extends SequentialCommandGroup {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public DropWobbleGoal(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;

        addCommands(new ReleaseWobble(wobbleGoalManipulator), new WaitCommand(500), new TuckArm(wobbleGoalManipulator));

        addRequirements(wobbleGoalManipulator);
    }
}
