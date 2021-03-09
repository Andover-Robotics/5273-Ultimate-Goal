package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRing;
import org.firstinspires.ftc.teamcode.commands.shooting.StartShooter;
import org.firstinspires.ftc.teamcode.commands.shooting.StopShooter;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.DropWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GrabWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.LowerArm;


@Autonomous(name = "Main Autonomous", group = "AA")
public class MainAuto extends AutonomousMaster {
    @Override
    public void initialize() {
        // Call the parent's initialize(), which instantiates our MecanumDriveSubsystem drive
        super.initialize();

        ParallelCommandGroup prepareToShoot = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(GlobalConfig.STARTING_POSITION)
                        .lineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION)
                        .build()
        ), new StartShooter(shooter));

        int ringShotDelay = 1000;

        SequentialCommandGroup shootRings = new SequentialCommandGroup(
                new ShootRing(shooter, cartridge), new WaitCommand(ringShotDelay),
                new ShootRing(shooter, cartridge), new WaitCommand((int) (ringShotDelay * 1.125 + 0.5)),
                new ShootRing(shooter, cartridge), new WaitCommand((int) (ringShotDelay * 1.25 + 0.5))
        );

        Pose2d thisDeliveryPoint;

        switch (ringStackResult) {
            case ONE:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_B;
                break;
            case FOUR:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_C;
                break;
            default:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_A;
        }

        SequentialCommandGroup deliverWobbleGoal = new SequentialCommandGroup(new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(GlobalConfig.RING_SHOOTING_POSITION)
                        .lineToLinearHeading(thisDeliveryPoint)
                        .build())
        );

        ParallelCommandGroup dropWobbleAndHeadToOther = new ParallelCommandGroup(
                new DropWobbleGoal(wobbleGoalManipulator).andThen(new LowerArm(wobbleGoalManipulator)),
                new WaitCommand(250).andThen(new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(thisDeliveryPoint)
                                .splineToSplineHeading(GlobalConfig.COLLECT_OTHER_WOBBLE, Math.toRadians(180))
                                .build())
                ).andThen(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.COLLECT_OTHER_WOBBLE)
                        .back(4)
                        .build()))
        );

        TrajectoryFollowerCommand returnToDeliveryPoint = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(GlobalConfig.COLLECT_OTHER_WOBBLE.plus(new Pose2d(-4, 0, 0)))
                        .splineToSplineHeading(thisDeliveryPoint.plus(new Pose2d(-6, 0, 0)), Math.toRadians(0))
                        .build());


        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(prepareToShoot)
                .andThen(new WaitCommand(250))
                .andThen(shootRings)
                .andThen(new StopShooter(shooter))
                .andThen(deliverWobbleGoal)
                .andThen(new WaitCommand(500))
                .andThen(dropWobbleAndHeadToOther)
                .andThen(new WaitCommand(500))
                .andThen(new GrabWobbleGoal(wobbleGoalManipulator))
                .andThen(returnToDeliveryPoint)
                .andThen(new WaitCommand(300))
                .andThen(new DropWobbleGoal(wobbleGoalManipulator))
        );
    }
}
