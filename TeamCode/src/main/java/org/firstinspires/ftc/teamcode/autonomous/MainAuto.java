package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GripWobble;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.LowerArm;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.OpenClawWide;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;


@Autonomous(name = "Main Autonomous", group = "AA")
public class MainAuto extends AutonomousMaster {
    @Override
    public void initialize() {
        // Call the parent's initialize(), which instantiates our MecanumDriveSubsystem drive
        super.initialize();

        ParallelCommandGroup prepareToShoot = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                // Start heading of 0 corrects the spline to go backwards and not hit the ring stack
                drive.trajectoryBuilder(GlobalConfig.STARTING_POSITION, 0)
                        .splineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION, 0)
                        .build()
        ), new StartShooter(shooter));

        int ringShotDelay = 1000;

        SequentialCommandGroup shootRings = new SequentialCommandGroup(
                new ShootRing(shooter, cartridge), new WaitCommand(ringShotDelay),
                new ShootRing(shooter, cartridge), new WaitCommand((int) (ringShotDelay * 1.125 + 0.5)),
                new ShootRing(shooter, cartridge), new WaitCommand((int) (ringShotDelay * 1.25 + 0.5))
        );

        Pose2d thisDeliveryPoint;
        double deliveryToWobbleHeading, deliveryToWobbleEndTangent;

        switch (ringStackResult) {
            case ONE:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_B;
                deliveryToWobbleHeading = GlobalConfig.DELIVERY_POINT_B.getHeading();
                deliveryToWobbleEndTangent = Math.toRadians(180);
                break;
            case FOUR:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_C;
                deliveryToWobbleHeading = GlobalConfig.DELIVERY_POINT_C.getHeading();
                deliveryToWobbleEndTangent = Math.toRadians(180);
                break;
            default:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_A;
                deliveryToWobbleHeading = 0;
                deliveryToWobbleEndTangent = Math.toRadians(225);
        }

        SequentialCommandGroup deliverWobbleGoal = new SequentialCommandGroup(new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(GlobalConfig.RING_SHOOTING_POSITION)
                        .lineToLinearHeading(thisDeliveryPoint)
                        .build())
        );

        ParallelCommandGroup dropWobbleAndHeadToOther = new ParallelCommandGroup(
                new DropWobbleGoal(wobbleGoalManipulator).andThen(new LowerArm(wobbleGoalManipulator)),
                new WaitCommand(250).andThen(new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(thisDeliveryPoint, deliveryToWobbleHeading)
                                .splineToSplineHeading(GlobalConfig.COLLECT_OTHER_WOBBLE, deliveryToWobbleEndTangent)
                                .build())
                )
        );

        Pose2d wobbleCollectionPose = new Pose2d(
                GlobalConfig.COLLECT_OTHER_WOBBLE.getX(),
                GlobalConfig.COLLECT_OTHER_WOBBLE.getY() - GlobalConfig.DISTANCE_STRAFED_TO_WOBBLE,
                0
        );

        // Opens the claw wide, then strafes to the side and begins to grab for the wobble goal as it approaches
        SequentialCommandGroup collectOtherWobble = new SequentialCommandGroup(
                new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(GlobalConfig.COLLECT_OTHER_WOBBLE)
                                .lineToLinearHeading(wobbleCollectionPose).build()
                ), new OpenClawWide(wobbleGoalManipulator)
                .andThen(new WaitUntilCommand(() -> Math.abs(drive.getPoseEstimate().getY()) >= GlobalConfig.COLLECT_OTHER_WOBBLE.getY() - GlobalConfig.DISTANCE_STRAFED_TO_WOBBLE * 0.20))
                .andThen(new GrabWobbleGoal(wobbleGoalManipulator))
        );

        TrajectoryFollowerCommand returnToDeliveryPoint = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(wobbleCollectionPose, Math.toRadians(ringStackResult == RingStackDetector.RingStackResult.ONE ? 30 : 225))
                        .splineToSplineHeading(thisDeliveryPoint, 0)
                        .build());

        ParallelCommandGroup park = new ParallelCommandGroup(
                new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(thisDeliveryPoint)
                                .splineToSplineHeading(GlobalConfig.PARKING_POSITION, Math.toRadians(ringStackResult == RingStackDetector.RingStackResult.ZERO ? 0 : 180))
                        .build()
                )
        );

        // How many ms to wait after driving to a delivery point for the wobble goal to stop shaking
        int wobbleGoalTransportDelay = 500;

        // RUN AUTO
        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(prepareToShoot)
                .andThen(new WaitCommand(100))
                .andThen(shootRings)
                .andThen(new StopShooter(shooter))
                .andThen(deliverWobbleGoal)
                .andThen(new WaitCommand(wobbleGoalTransportDelay))
                .andThen(dropWobbleAndHeadToOther)
                .andThen(collectOtherWobble)
                .andThen(returnToDeliveryPoint)
                .andThen(new WaitCommand(wobbleGoalTransportDelay))
                .andThen(new DropWobbleGoal(wobbleGoalManipulator))
                .andThen(park)
        );
    }
}
