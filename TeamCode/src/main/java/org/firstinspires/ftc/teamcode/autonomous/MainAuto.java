package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.cartridge.LowerCartridge;
import org.firstinspires.ftc.teamcode.commands.cartridge.RaiseCartridge;
import org.firstinspires.ftc.teamcode.commands.intake.Outtake;
import org.firstinspires.ftc.teamcode.commands.intake.StartIntake;
import org.firstinspires.ftc.teamcode.commands.intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.shooting.ShootRings;
import org.firstinspires.ftc.teamcode.commands.shooting.StartShooter;
import org.firstinspires.ftc.teamcode.commands.shooting.StopShooter;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.DropWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.GrabWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.LowerArm;
import org.firstinspires.ftc.teamcode.commands.wobble_goal.OpenClawWide;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;


@Autonomous(name = "Main Autonomous", group = "AA")
public class MainAuto extends AutonomousMaster {
    @Override
    public void initialize() {
        // Call the parent's initialize(), which instantiates our MecanumDriveSubsystem drive
        super.initialize();

        ParallelCommandGroup prepareToShoot = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive,
                // Start heading of 0 corrects the spline to go backwards and not hit the ring stack
                drive.trajectoryBuilder(GlobalConfig.STARTING_POSITION)
                        .lineToConstantHeading(GlobalConfig.INTERMEDIATE_POSITION)
                        .splineToSplineHeading(GlobalConfig.RING_SHOOTING_POSITION, 45)
                        .build()
        ), new StartShooter(shooter, telemetry, true));

        int numRings = 3;
        //(ringStackResult == RingStackDetector.RingStackResult.FOUR) ? 3 : 4;
        int ringShotDelay = (ringStackResult == RingStackDetector.RingStackResult.ZERO) ? 750 : 500;

        Pose2d thisDeliveryPoint;
        double deliveryToWobbleHeading, deliveryToWobbleEndTangent;

        switch (ringStackResult) {
            case ONE:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_B;
                deliveryToWobbleHeading = GlobalConfig.DELIVERY_POINT_B.getHeading();
                deliveryToWobbleEndTangent = Math.toRadians(200);
                break;
            case FOUR:
                thisDeliveryPoint = GlobalConfig.DELIVERY_POINT_C;
                deliveryToWobbleHeading = GlobalConfig.DELIVERY_POINT_C.getHeading();
                deliveryToWobbleEndTangent = Math.toRadians(200);
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
                        .build())
                //new RaiseArm(wobbleGoalManipulator)

        );

        Pose2d wobbleCollectionPose = new Pose2d(
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getX(),
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getY(),
                (ringStackResult == RingStackDetector.RingStackResult.FOUR ? GlobalConfig.COLLECT_OTHER_WOBBLE_FOUR_RINGS : (ringStackResult == RingStackDetector.RingStackResult.ONE ? GlobalConfig.COLLECT_OTHER_WOBBLE_ONE_RING : GlobalConfig.COLLECT_OTHER_WOBBLE)).getHeading()
        );

        ParallelCommandGroup startIntake = new ParallelCommandGroup(new StartIntake(intake), new LowerCartridge(cartridge));

        TrajectoryFollowerCommand deliverWobbleGoalTrajectory = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(thisDeliveryPoint, deliveryToWobbleHeading)
                        .splineToSplineHeading(wobbleCollectionPose, deliveryToWobbleEndTangent)
                        .build());

        ParallelCommandGroup dropWobbleAndHeadToOther = new ParallelCommandGroup(
                new LowerArm(wobbleGoalManipulator)
                        .andThen(new DropWobbleGoal(wobbleGoalManipulator))
                        .andThen(new OpenClawWide(wobbleGoalManipulator))
                        .andThen(new LowerArm(wobbleGoalManipulator))
        );

        dropWobbleAndHeadToOther.addCommands(ringStackResult != RingStackDetector.RingStackResult.ZERO ? deliverWobbleGoalTrajectory.andThen(startIntake) : deliverWobbleGoalTrajectory);


        //SequentialCommandGroup linetoPosition= new SequentialCommandGroup(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(wobbleCollectionPose.plus(new Pose2d(-1*offset, (ringStackResult== RingStackDetector.RingStackResult.ZERO)? offset: -1*offset, Math.toRadians(0.0)))).lineToConstantHeading(new Vector2d(wobbleCollectionPose.getX(), wobbleCollectionPose.getY())).build()));
        //.splineToSplineHeading(wobbleCollectionPose.plus(new Pose2d(offset, (ringStackResult== RingStackDetector.RingStackResult.ZERO)? -1/2 * offset: offset/2, Math.toRadians(0.0))), deliveryToWobbleEndTangent)
        //                                .splineToConstantHeading(new Vector2d(wobbleCollectionPose.getX(), wobbleCollectionPose.getY()), Math.toRadians(180.0))
        SequentialCommandGroup grabWobble = new GrabWobbleGoal(wobbleGoalManipulator);

        double startingHeading;
        int xOffset, yOffset;

        switch (ringStackResult) {
            case ONE:
                startingHeading = Math.toRadians(210);
                xOffset = -5;
                yOffset = 4;
                break;
            case FOUR:
                startingHeading = - Math.toRadians(60);
                xOffset = -9;
                yOffset = -8;
                break;
            default:
                startingHeading = Math.toRadians(225);
                xOffset = -12;
                yOffset = 0;
                break;
        }

        /*TrajectoryFollowerCommand strafeRight = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(wobbleCollectionPose).
                        strafeRight(Math.abs(GlobalConfig.INTAKE_POSITION.getY()-wobbleCollectionPose.getY())).build());
                        */

        TrajectoryFollowerCommand intakePosition = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(wobbleCollectionPose).splineToLinearHeading(GlobalConfig.INTAKE_POSITION, Math.toRadians(0.0)).build());

        int outtakeDuration = 625;
        double targetX = GlobalConfig.INTAKE_POSITION.getX() + GlobalConfig.STRAFE_DISTANCE;
        double stopPoint = 0.95;
        ParallelCommandGroup stopIntake = new ParallelCommandGroup(new StopIntake(intake), new RaiseCartridge(cartridge));
        SequentialCommandGroup strafeIntake = new SequentialCommandGroup(
                // Drive forward, then back up briefly and drive forward again - gets the rings unstuck from the front barrier, which sometimes happens
                new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION).forward(GlobalConfig.STRAFE_DISTANCE * 0.4).build()),
                new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE * 0.4, 0, 0))).back(GlobalConfig.STRAFE_DISTANCE / 65).build()),
                new ParallelCommandGroup(
                        new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE * 25 / 65.0, 0, 0))).forward(GlobalConfig.STRAFE_DISTANCE * 40 / 63.0).build()),
                        // Stop intaking stopPoint % of the way through so that the fourth ring isn't taken in
                        new WaitUntilCommand(() -> Math.abs(drive.getPoseEstimate().getX() - targetX) < (1 - stopPoint) * GlobalConfig.STRAFE_DISTANCE)
                                .andThen(new WaitCommand(50))
                                .andThen(new Outtake(intake))
                                .andThen(new WaitCommand(outtakeDuration))
                                .andThen(stopIntake)
                                .andThen(new RaiseCartridge(cartridge))
                )
//                        .andThen(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE * 41 / 63.0, 0, 0))).back(GlobalConfig.STRAFE_DISTANCE / 63).build()))
//                        .andThen(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE * 40 / 63.0, 0, 0))).forward(GlobalConfig.STRAFE_DISTANCE * 23 / 63.0).build())),
//                .andThen(new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE / 2, 0, 0))).forward(STRAFE_DISTANCE / 2).build())),
//                new WaitUntilCommand(() -> Math.abs(drive.getPoseEstimate().getX() - GlobalConfig.INTAKE_POSITION.getX()) > shooterStopPoint * GlobalConfig.STRAFE_DISTANCE)

        );


         TrajectoryFollowerCommand returnToDeliveryPoint = new TrajectoryFollowerCommand(drive,
                    drive.trajectoryBuilder((ringStackResult != RingStackDetector.RingStackResult.ZERO) ? GlobalConfig.RING_SHOOTING_POSITION.plus(new Pose2d(-2.0, 0.0, Math.toRadians(-1.5))) : wobbleCollectionPose, startingHeading)
                            .splineToSplineHeading(thisDeliveryPoint.plus(new Pose2d(
                                    xOffset,
                                    yOffset,
                                    Math.toRadians(0.0)
                            )), Math.toRadians(0.0))
                            .build());

        ParallelCommandGroup prepareToHighGoal = new ParallelCommandGroup(new TrajectoryFollowerCommand(drive, drive
                .trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE, 0, 0)))
                .splineToLinearHeading(GlobalConfig.RING_SHOOTING_POSITION.plus(new Pose2d(0.0, 0.0, Math.toRadians(-1.5))), Math.toRadians(0.0)).build()),
                new StartShooter(shooter, telemetry, true)
        );



        //new Pose2d(-2.0, 0.0, 0.0)

        //new TrajectoryFollowerCommand(drive, drive
        //                .trajectoryBuilder(GlobalConfig.INTAKE_POSITION.plus(new Pose2d(GlobalConfig.STRAFE_DISTANCE, 0, 0)))
        //                .splineToConstantHeading(new Vector2d(GlobalConfig.RING_SHOOTING_POSITION.getX() + 2.0, GlobalConfig.RING_SHOOTING_POSITION.getY()), Math.toRadians(0.0)).build())


        TrajectoryFollowerCommand parking = new TrajectoryFollowerCommand(drive,
                drive.trajectoryBuilder(thisDeliveryPoint.plus(new Pose2d(
                        xOffset,
                        yOffset
                )))
                        .strafeTo(GlobalConfig.PARKING_POSITION)
                        .build());

        ParallelCommandGroup park = new ParallelCommandGroup(parking);


        // How many ms to wait after driving to a delivery point for the wobble goal to stop shaking
        //int wobbleGoalTransportDelay = 250;

        // RUN AUTO
        if (ringStackResult == RingStackDetector.RingStackResult.ZERO) {
            schedule(new WaitUntilCommand(this::isStarted)
                    .andThen(prepareToShoot)
                    .andThen(new ShootRings(shooter, cartridge, numRings, ringShotDelay, telemetry, true))
                    .andThen(new StopShooter(shooter))
                    .andThen(deliverWobbleGoal)
                    .andThen(dropWobbleAndHeadToOther)
                    //.andThen(new WaitCommand(wobbleGoalTransportDelay))
                    .andThen(grabWobble)
                    .andThen(returnToDeliveryPoint)
                    //.andThen(new WaitCommand(wobbleGoalTransportDelay))
                    .andThen(new DropWobbleGoal(wobbleGoalManipulator))
                    .andThen(park)
                    .andThen(new InstantCommand(() -> PoseStorage.currentPose = drive.getPoseEstimate()))
            );
        } else {
            schedule(new WaitUntilCommand(this::isStarted)
                            .andThen(prepareToShoot)
                            .andThen(new ShootRings(shooter, cartridge, numRings, ringShotDelay, telemetry, true))
                            .andThen(new WaitCommand(ringShotDelay / 4).andThen(new StopShooter(shooter)))
                            .andThen(deliverWobbleGoal)
                            //.andThen(new WaitCommand(wobbleGoalTransportDelay))
                            .andThen(dropWobbleAndHeadToOther)
                            .andThen(grabWobble)
                            //.andThen(strafeRight)]
                            .andThen(intakePosition)
                            .andThen(strafeIntake)
                            .andThen(prepareToHighGoal)
//                    .andThen(new StopIntake(intake))
//                            .andThen(new TurnCommand(drive, GlobalConfig.RING_SHOOTING_POSITION.getHeading() - Math.toRadians(1.0)))
                            .andThen(new ShootRings(shooter, cartridge, (ringStackResult == RingStackDetector.RingStackResult.ONE) ? numRings + 1 : numRings + 1, ringShotDelay, telemetry, true))
                            .andThen(new ParallelCommandGroup(returnToDeliveryPoint, new StopShooter(shooter), new LowerCartridge(cartridge).andThen(new WaitCommand(250)).andThen(new StartIntake(intake))))
                            //.andThen(new WaitCommand(wobbleGoalTransportDelay))
                            .andThen(new LowerArm(wobbleGoalManipulator).andThen(new DropWobbleGoal(wobbleGoalManipulator)))
                            .andThen(park)
                            .andThen(new InstantCommand(() -> PoseStorage.currentPose = drive.getPoseEstimate()))
            );

        }
    }
}