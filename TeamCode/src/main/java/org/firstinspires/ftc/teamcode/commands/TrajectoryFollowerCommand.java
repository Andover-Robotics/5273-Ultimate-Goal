package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class TrajectoryFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(MecanumDriveSubsystem drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
        // Update the PoseStorage as we move to maintain an accurate approximation in case of OpMode crash
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
