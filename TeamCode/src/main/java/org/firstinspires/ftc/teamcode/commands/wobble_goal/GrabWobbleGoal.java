package org.firstinspires.ftc.teamcode.commands.wobble_goal;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;

public class GrabWobbleGoal extends SequentialCommandGroup {
    private final WobbleGoalManipulatorSubsystem wobbleGoalManipulator;

    public GrabWobbleGoal(WobbleGoalManipulatorSubsystem wobbleGoalManipulator) {
        this.wobbleGoalManipulator = wobbleGoalManipulator;

        addCommands(new GripWobble(wobbleGoalManipulator), new WaitCommand(400), new RaiseArm(wobbleGoalManipulator));

        addRequirements(wobbleGoalManipulator);
    }
}
