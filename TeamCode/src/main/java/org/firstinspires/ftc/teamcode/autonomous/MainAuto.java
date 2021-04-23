package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRing;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRings;
import org.firstinspires.ftc.teamcode.commands.shooting.StartShooter;
import org.firstinspires.ftc.teamcode.commands.shooting.StopShooter;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.DropWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GrabWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GripWobble;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.LowerArm;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.OpenClawWide;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.RaiseArm;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

import static org.firstinspires.ftc.teamcode.GlobalConfig.MM_PER_INCH;
import static org.firstinspires.ftc.teamcode.GlobalConfig.ROBOT_LENGTH_MM;
import static org.firstinspires.ftc.teamcode.GlobalConfig.ROBOT_WIDTH_MM;


@Autonomous(name = "Main Autonomous", group = "AA")
public class MainAuto extends AutonomousMaster {
    @Override
    public void initialize() {
        // Call the parent's initialize(), which instantiates our MecanumDriveSubsystem drive
        super.initialize();

        ParallelCommandGroup prepareToShoot = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                // Start heading of 0 corrects the spline to go backwards and not hit the ring stack
                drive.trajectoryBuilder(GlobalConfig.STARTING_POSITION, 0)
                        .splineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION, 45)
                        .build()
        ), new StartShooter(shooter, telemetry));

        int numRings = 4;
        //(ringStackResult == RingStackDetector.RingStackResult.FOUR) ? 3 : 4;
        int ringShotDelay = 1250;

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
                deliveryToWobbleHeading = Math.toRadians(135);
                deliveryToWobbleEndTangent = Math.toRadians(180);
        }

        ParallelCommandGroup deliverWobbleGoal = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(GlobalConfig.RING_SHOOTING_POSITION.plus(new Pose2d(
                        0,
                        0,
                        35
                )))
                        .lineToLinearHeading(thisDeliveryPoint)
                        .build()),
                new RaiseArm(wobbleGoalManipulator)

        );

        Pose2d wobbleCollectionPose = new Pose2d(
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getX(),
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getY(),
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getHeading()
        );

        ParallelCommandGroup dropWobbleAndHeadToOther = new ParallelCommandGroup(
                new DropWobbleGoal(wobbleGoalManipulator)
                        .andThen(new LowerArm(wobbleGoalManipulator))
                        .andThen(new WaitCommand(250))
                        .andThen(new OpenClawWide(wobbleGoalManipulator)),
                new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(thisDeliveryPoint, deliveryToWobbleHeading)
                                .splineToSplineHeading(wobbleCollectionPose, deliveryToWobbleEndTangent)
                                .build()
                )
        );

        SequentialCommandGroup grabWobble = new SequentialCommandGroup(new WaitCommand(250).andThen(new GrabWobbleGoal(wobbleGoalManipulator)));

        // Opens the claw wide, then strafes to the side and begins to grab for the wobble goal as it approaches

        double startingHeading;
        int xOffset, yOffset;

        switch (ringStackResult) {
            case ONE:
                startingHeading = Math.toRadians(30);
                xOffset = -10;
                yOffset = -3;
                break;
            case FOUR:
                startingHeading = Math.toRadians(200);
                xOffset = -12;
                yOffset = -6;
                break;
            default:
                startingHeading = Math.toRadians(225);
                xOffset = -12;
                yOffset = 0;
                break;
        }

        TrajectoryFollowerCommand returnToDeliveryPoint = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(wobbleCollectionPose, Math.toRadians(startingHeading))
                        .splineToSplineHeading(thisDeliveryPoint.plus(new Pose2d(
                                xOffset,
                                yOffset
                        )), 0)
                        .build());

        ParallelCommandGroup park = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(thisDeliveryPoint.plus(new Pose2d(
                        xOffset,
                        yOffset
                )))
                        .strafeTo(GlobalConfig.PARKING_POSITION)
                        .build()
        ));

        // How many ms to wait after driving to a delivery point for the wobble goal to stop shaking
        int wobbleGoalTransportDelay = 250;

        // RUN AUTO
        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(prepareToShoot)
                .andThen(new WaitCommand(100))
                .andThen(new ShootRings(shooter, cartridge, numRings, ringShotDelay, telemetry))
                .andThen(new StopShooter(shooter))
                .andThen(deliverWobbleGoal)
                .andThen(new WaitCommand(wobbleGoalTransportDelay))
                .andThen(dropWobbleAndHeadToOther)
                .andThen(grabWobble)
                .andThen(returnToDeliveryPoint)
                .andThen(new WaitCommand(wobbleGoalTransportDelay))
                .andThen(new DropWobbleGoal(wobbleGoalManipulator))
                .andThen(park)
                .andThen(new InstantCommand(() -> PoseStorage.currentPose = drive.getPoseEstimate()))
        );
    }
}
