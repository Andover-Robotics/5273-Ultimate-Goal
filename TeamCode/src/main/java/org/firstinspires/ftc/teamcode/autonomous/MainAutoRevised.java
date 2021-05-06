package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.cartridge.LowerCartridge;
import org.firstinspires.ftc.teamcode.commands.cartridge.RaiseCartridge;
import org.firstinspires.ftc.teamcode.commands.intake.StartIntake;
import org.firstinspires.ftc.teamcode.commands.intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRing;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRings;
import org.firstinspires.ftc.teamcode.commands.shooting.StartShooter;
import org.firstinspires.ftc.teamcode.commands.shooting.StopShooter;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.DropWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GrabWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.LowerArm;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.OpenClawWide;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.RaiseArm;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

class MainAutoRevised extends AutonomousMaster {
    @Override
    public void initialize() {
        // Call the parent's initialize(), which instantiates our MecanumDriveSubsystem drive
        super.initialize();

        ParallelCommandGroup prepareToPowerShot = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                // Start heading of 0 corrects the spline to go backwards and not hit the ring stack
                drive.trajectoryBuilder(GlobalConfig.STARTING_POSITION)
                        .lineToConstantHeading(GlobalConfig.INTERMEDIATE_POSITION)
                        .splineToSplineHeading(GlobalConfig.POWER_SHOT_SHOOTING_POSITION, 45)
                        .build()),
                new StartShooter(shooter, telemetry, false));

        int numRings = 3;
        //(ringStackResult == RingStackDetector.RingStackResult.FOUR) ? 3 : 4;
        int ringShotDelay = 1250;
        int turnDelay = 1000;
        double angleInterval = Math.toRadians(15.0);

        ParallelCommandGroup shootPowerShots = new ParallelCommandGroup(new WaitCommand(ringShotDelay).andThen(new TurnCommand(drive, angleInterval)).andThen(new WaitCommand(turnDelay)).andThen(new TurnCommand(drive, angleInterval)), new ShootRings(shooter, cartridge, numRings, ringShotDelay, telemetry, false));

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

        ParallelCommandGroup dropWobbleAndHeadToOther;
        ParallelCommandGroup startIntake = new ParallelCommandGroup(new StartIntake(intake), new LowerCartridge(cartridge));
        dropWobbleAndHeadToOther = new ParallelCommandGroup(
                new DropWobbleGoal(wobbleGoalManipulator)
                        .andThen(new RaiseArm(wobbleGoalManipulator))
                        .andThen(new WaitCommand(500))
                        .andThen(new OpenClawWide(wobbleGoalManipulator)),
                new TrajectoryFollowerCommand(drive,
                        drive.trajectoryBuilder(thisDeliveryPoint, deliveryToWobbleHeading)
                                .splineToSplineHeading(wobbleCollectionPose, deliveryToWobbleEndTangent)
                                .build())
        );

        //SequentialCommandGroup linetoPosition= new SequentialCommandGroup(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(wobbleCollectionPose.plus(new Pose2d(-1*offset, (ringStackResult== RingStackDetector.RingStackResult.ZERO)? offset: -1*offset, Math.toRadians(0.0)))).lineToConstantHeading(new Vector2d(wobbleCollectionPose.getX(), wobbleCollectionPose.getY())).build()));
        //.splineToSplineHeading(wobbleCollectionPose.plus(new Pose2d(offset, (ringStackResult== RingStackDetector.RingStackResult.ZERO)? -1/2 * offset: offset/2, Math.toRadians(0.0))), deliveryToWobbleEndTangent)
        //                                .splineToConstantHeading(new Vector2d(wobbleCollectionPose.getX(), wobbleCollectionPose.getY()), Math.toRadians(180.0))
        SequentialCommandGroup grabWobble = new SequentialCommandGroup(new LowerArm(wobbleGoalManipulator), new WaitCommand(750).andThen(new GrabWobbleGoal(wobbleGoalManipulator)));

        double startingHeading;
        int xOffset, yOffset;

        switch (ringStackResult) {
            case ONE:
                startingHeading = Math.toRadians(30);
                xOffset = -10;
                yOffset = 4;
                break;
            case FOUR:
                startingHeading = Math.toRadians(200);
                xOffset = -12;
                yOffset = -8;
                break;
            default:
                startingHeading = Math.toRadians(225);
                xOffset = -12;
                yOffset = 0;
                break;
        }

        TrajectoryFollowerCommand intakePosition = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(wobbleCollectionPose)
                        .splineToSplineHeading(GlobalConfig.INTAKE_POSITION, Math.toRadians(0.0)).build());

        int intakeDelay = 1250;
        ParallelCommandGroup stopIntake = new ParallelCommandGroup(new StopIntake(intake), new RaiseCartridge(cartridge));
        TrajectoryFollowerCommand strafeIntake = new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION).forward(GlobalConfig.STRAFE_DISTANCE).build());

        TrajectoryFollowerCommand returnToDeliveryPoint = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder((ringStackResult == RingStackDetector.RingStackResult.ONE) ? GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE, 0, 0)) : wobbleCollectionPose, Math.toRadians(startingHeading))
                        .splineToSplineHeading(thisDeliveryPoint.plus(new Pose2d(
                                xOffset,
                                yOffset
                        )), 0)
                        .build());

        ParallelCommandGroup prepareToHighGoal = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive, drive
                .trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE, 0, 0)))
                .splineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION, Math.toRadians(0.0)).build()),
                new StartShooter(shooter, telemetry, true));

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
        if (!(ringStackResult == RingStackDetector.RingStackResult.ONE)) {
            schedule(new WaitUntilCommand(this::isStarted)
                    .andThen(prepareToPowerShot)
                    .andThen(new WaitCommand(100))
                    .andThen(shootPowerShots)
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
        } else {
            schedule(new WaitUntilCommand(this::isStarted)
                    .andThen(prepareToPowerShot)
                    .andThen(new WaitCommand(100))
                    .andThen(shootPowerShots)
                    .andThen(new StopShooter(shooter))
                    .andThen(deliverWobbleGoal)
                    .andThen(new WaitCommand(wobbleGoalTransportDelay))
                    .andThen(dropWobbleAndHeadToOther)
                    .andThen(grabWobble)
                    .andThen(startIntake)
                    .andThen(intakePosition)
                    .andThen(strafeIntake)
                    .andThen(stopIntake)
                    .andThen(prepareToHighGoal)
                    .andThen(new ShootRing(shooter, cartridge, telemetry, true))
                    .andThen(returnToDeliveryPoint)
                    .andThen(new WaitCommand(wobbleGoalTransportDelay))
                    .andThen(new DropWobbleGoal(wobbleGoalManipulator))
                    .andThen(park)
                    .andThen(new InstantCommand(() -> PoseStorage.currentPose = drive.getPoseEstimate()))
            );

        }
    }
}
