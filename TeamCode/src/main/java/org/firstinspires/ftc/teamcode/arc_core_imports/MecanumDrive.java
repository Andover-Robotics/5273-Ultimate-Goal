package org.firstinspires.ftc.teamcode.arc_core_imports;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Implements the {@link StrafingDriveTrain} for a Mecanum drivetrain. <p> See {@link
 * #fromOctagonalMotors(DcMotor, DcMotor, DcMotor, DcMotor, OpMode, int, int)} and {@link
 * #fromCrossedMotors(DcMotor, DcMotor, DcMotor, DcMotor, OpMode, int, int)} for instructions about
 * easier construction.
 */
public class MecanumDrive extends StrafingDriveTrain {

    private final MotorAdapter
            motorLeftDiagonalLeftSide, motorRightDiagonalLeftSide,
            motorLeftDiagonalRightSide, motorRightDiagonalRightSide;
    private final int ticksPerInch, ticksPer360;

    /**
     * Constructs a new <code>MecanumDrive</code> instance with the given {@link DcMotor}s and encoder
     * parameters.
     *
     * @param leftDiagLeftSide The motor that is on the left side of the robot and (when powered positively) causes the robot to strafe to the front-left diagonal
     * @param leftDiagRightSide The motor that is on the right side of the robot and (when powered positively) causes the robot to strafe to the front-left diagonal
     * @param rightDiagLeftSide The motor that is on the left side of the robot and (when powered positively) causes the robot to strafe to the front-right diagonal
     * @param rightDiagRightSide The motor that is on the right side of the robot and (when powered positively) causes the robot to strafe to the front-right diagonal
     * @param opMode The main {@link OpMode}
     * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
     * inch for the robot
     * @param ticksPer360 The number of encoder ticks required to cause a full rotation for the robot,
     * when this amount is applied to the left and right sides in opposite directions
     */
    public MecanumDrive(MotorAdapter leftDiagLeftSide, MotorAdapter leftDiagRightSide,
                        MotorAdapter rightDiagLeftSide, MotorAdapter rightDiagRightSide,
                        OpMode opMode, int ticksPerInch, int ticksPer360) {

        super(opMode);

        motorLeftDiagonalLeftSide = leftDiagLeftSide;
        motorLeftDiagonalRightSide = leftDiagRightSide;
        motorRightDiagonalLeftSide = rightDiagLeftSide;
        motorRightDiagonalRightSide = rightDiagRightSide;

        this.ticksPerInch = ticksPerInch;
        this.ticksPer360 = ticksPer360;
    }

    /**
     * Constructs a new MecanumDrive instance that uses the given physical motors, which are arranged
     * in an octagonal manner. <p> <h2>The Octagonal Configuration</h2> In the octagonal
     * configuration, the front left and the back right motors cause the robot to move forward and to
     * the right (diagonally), and the other two motors cause the robot to move forward and to the
     * left.
     *
     * It is named to be "octagonal" because of its diagram:
     * <pre>
     *   /-\
     *   | |
     *   \-/
     * </pre>
     *
     * @param motorFL The motor located at the front-left of the robot
     * @param motorFR The motor located at the front-right of the robot
     * @param motorBL The motor located at the rear-left of the robot
     * @param motorBR The motor located at the rear-right of the robot
     * @param opMode The main {@link OpMode}
     * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
     * inch for the robot
     * @param ticksPer360 The number of encoder ticks required to cause a full rotation for the robot,
     * when this amount is applied to the left and right sides in opposite directions
     * @return The new MecanumDrive instance
     */
    public static MecanumDrive fromOctagonalMotors(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL,
                                                   DcMotor motorBR,
                                                   OpMode opMode, int ticksPerInch, int ticksPer360) {
        // /-\
        // | |
        // \-/

        return new MecanumDrive(new MotorAdapter(motorBL), new MotorAdapter(motorFR),
                new MotorAdapter(motorFL), new MotorAdapter(motorBR), opMode, ticksPerInch, ticksPer360);
    }

    /**
     * Constructs a new MecanumDrive instance that uses the given physical motors, which are arranged
     * in a a crossed manner. <p> <b>The Crossed Configuration</b> In the crossed configuration, the
     * front left and the back right motors cause the robot to move forward and to the left
     * (diagonally), and the other two motors cause the robot to move forward and to the right.
     *
     * It is named to be "crossed" because of its diagram:
     * <pre>
     *   \-/
     *   | |
     *   /-\
     * </pre>
     *
     * @param motorFL The motor located at the front-left of the robot
     * @param motorFR The motor located at the front-right of the robot
     * @param motorBL The motor located at the rear-left of the robot
     * @param motorBR The motor located at the rear-right of the robot
     * @param opMode The main {@link OpMode}
     * @param ticksPerInch The number of encoder ticks required to cause a diagonal displacement of 1
     * inch for the robot
     * @param ticksPer360 The number of encoder ticks required to cause a full rotation for the robot,
     * when this amount is applied to the left and right sides in opposite directions
     * @return The new MecanumDrive instance
     */
    public static MecanumDrive fromCrossedMotors(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL,
                                                 DcMotor motorBR,
                                                 OpMode opMode, int ticksPerInch, int ticksPer360) {
        // \-/
        // | |
        // /-\

        return new MecanumDrive(
                new MotorAdapter(motorFL), new MotorAdapter(motorBR),
                new MotorAdapter(motorBL), new MotorAdapter(motorFR), opMode, ticksPerInch, ticksPer360);
    }

    // Rotates the given displacement by 45deg clockwise, assigns its components to the diagonals as
    // a tick offset (setTargetPosition), then scales its components down such that the greatest
    // component is equal to the power given, followed by assigning these components to the diagonals
    // as power.
    private void driveWithEncoder(Coordinate displacement, double power) {
        double clippedPower = Range.clip(power, -1, 1);

        if (displacement.getPolarDistance() < 1e-5) {
            return;
        }

        Coordinate diagonalOffsets = displacement.rotate(-45);
        double maxOffset = Math.max(diagonalOffsets.getX(), diagonalOffsets.getY());

        int leftOffset = (int) (diagonalOffsets.getY() * ticksPerInch),
                rightOffset = (int) (diagonalOffsets.getX() * ticksPerInch);

        double leftPower = Math.abs(clippedPower * (diagonalOffsets.getY() / maxOffset)),
                rightPower = Math.abs(clippedPower * (diagonalOffsets.getX() / maxOffset));

        for (MotorAdapter motor : leftDiagonal())
            motor.startRunToPosition(leftOffset, leftPower);
        for (MotorAdapter motor : rightDiagonal())
            motor.startRunToPosition(rightOffset, rightPower);

        while (isBusy() && opModeIsActive()) {
        }

        stop();
    }

    @Override
    public void rotateClockwise(int degrees, double power) {
        rotateWithEncoder(-Converter.normalizedDegrees(degrees), -Math.abs(power));
    }

    @Override
    public void rotateCounterClockwise(int degrees, double power) {
        rotateWithEncoder(Converter.normalizedDegrees(degrees), Math.abs(power));
    }

    // Positive input means counter-clockwise
    private void rotateWithEncoder(double degrees, double power) {
        double clippedPower = Math.abs(Range.clip(power, -1, 1));
        double rotationTicks = degrees / 360.0 * ticksPer360;

        for (MotorAdapter motor : leftSide())
            motor.startRunToPosition((int) -rotationTicks, clippedPower);
        for (MotorAdapter motor : rightSide())
            motor.startRunToPosition((int) rotationTicks, clippedPower);


        while (isBusy() && opModeIsActive()) {
        }

        stop();
        setMotorMode(RUN_USING_ENCODER);
    }

    private MotorAdapter[] leftSide() {
        return new MotorAdapter[] {
                motorRightDiagonalLeftSide,
                motorLeftDiagonalLeftSide
        };
    }

    private MotorAdapter[] rightSide() {
        return new MotorAdapter[] {
                motorRightDiagonalRightSide,
                motorLeftDiagonalRightSide
        };
    }

    private MotorAdapter[] leftDiagonal() {
        return new MotorAdapter[] {
                motorLeftDiagonalLeftSide,
                motorLeftDiagonalRightSide
        };
    }

    private MotorAdapter[] rightDiagonal() {
        return new MotorAdapter[] {
                motorRightDiagonalLeftSide,
                motorRightDiagonalRightSide
        };
    }

    private MotorAdapter[] allMotors() {
        return new MotorAdapter[] {
                motorRightDiagonalRightSide,
                motorRightDiagonalLeftSide,
                motorLeftDiagonalRightSide,
                motorLeftDiagonalLeftSide
        };
    }

    @Override
    public void strafeInches(Coordinate inchOffset, double power) {
        driveWithEncoder(inchOffset, power);
    }

    // -- TeleOp methods --

    @Override
    public void setMovementPower(double power) {
        double clippedPower = Range.clip(power, -1, 1);

        setMotorMode(RUN_WITHOUT_ENCODER);

        for (MotorAdapter motor : allMotors()) {
            motor.setPower(clippedPower);
        }
    }

    @Override
    public void setRotationPower(double power) { //clockwise if power is positive
        double clippedPower = Range.clip(power, -1, 1);

        setMotorMode(RUN_WITHOUT_ENCODER);

        for (MotorAdapter motor : leftSide()) {
            motor.setPower(clippedPower);
        }
        for (MotorAdapter motor : rightSide()) {
            motor.setPower(-clippedPower);
        }
    }

    @Override
    public void setStrafeRotation(Coordinate direction, double strafePower, double z) {
        Coordinate xy = direction.getPolarDistance() < 1e-5 ? Coordinate.fromXY(0, 0) :
                Coordinate.fromPolar(1, direction.getPolarDirection());
        double x = xy.getX(), y = xy.getY(),
                rightDiagonalLeftSide = x + y + z,
                rightDiagonalRightSide = x + y - z,
                leftDiagonalLeftSide = -x + y + z,
                leftDiagonalRightSide = -x + y - z,
                maxPower = Math.max(Math.max(Math.abs(rightDiagonalLeftSide), Math.abs(rightDiagonalRightSide)),
                        Math.max(Math.abs(leftDiagonalLeftSide), Math.abs(leftDiagonalRightSide))),
                coefficient = maxPower == 0 ? 0 : Math.abs(strafePower) / maxPower;

        rightDiagonalLeftSide *= coefficient;
        rightDiagonalRightSide *= coefficient;
        leftDiagonalLeftSide *= coefficient;
        leftDiagonalRightSide *= coefficient;

        for (MotorAdapter motor : allMotors()) {
            motor.setMode(RUN_WITHOUT_ENCODER);
        }

        motorRightDiagonalLeftSide.setPower(rightDiagonalLeftSide);
        motorRightDiagonalRightSide.setPower(rightDiagonalRightSide);
        motorLeftDiagonalLeftSide.setPower(leftDiagonalLeftSide);
        motorLeftDiagonalRightSide.setPower(leftDiagonalRightSide);
    }

    @Override
    public void setMovementAndRotation(double movePower, double rotatePower) {
        setMotorMode(RUN_WITHOUT_ENCODER);

        double leftPower = movePower + rotatePower,
                rightPower = movePower - rotatePower;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        for (MotorAdapter motor : leftSide()) {
            motor.setPower(leftPower);
        }
        for (MotorAdapter motor : rightSide()) {
            motor.setPower(rightPower);
        }
    }

    @Override
    protected IMotor[] getMotors() {
        return new MotorAdapter[] {
                motorLeftDiagonalLeftSide,
                motorLeftDiagonalRightSide,
                motorRightDiagonalLeftSide,
                motorRightDiagonalRightSide
        };
    }
}
