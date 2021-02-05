package org.firstinspires.ftc.teamcode.autonomous.alliance_insignificant;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Parking Auto", group = "AA")
public class ParkingAuto extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStartWithPings();

        drive.driveRobotCentric(0, 0.75, 0);

        sleep(1500);

        drive.stop();

        stop();
    }

    private void setup() {
        MotorEx motorFL = new MotorEx(hardwareMap, "motorFL", Motor.GoBILDA.RPM_312);
        MotorEx motorFR = new MotorEx(hardwareMap, "motorFR", Motor.GoBILDA.RPM_312);
        MotorEx motorBL = new MotorEx(hardwareMap, "motorBL", Motor.GoBILDA.RPM_312);
        MotorEx motorBR = new MotorEx(hardwareMap, "motorBR", Motor.GoBILDA.RPM_312);

        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(motorFL, motorFR, motorBL, motorBR);
    }

    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Waiting in Init", System.currentTimeMillis());
            telemetry.update();
        }

        telemetry.update();
    }
}
