package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonomousMaster extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStartWithPings();

        telemetry.addData("Status", "STARTING");
        telemetry.update();
    }

    protected void checkForStop() throws InterruptedException {
        if (isStopRequested())
            throw new InterruptedException();
    }

    protected void initialize() {

    }

    // Waits for the OpMode to be run while sending messages between the phones
    // This avoids timeouts and resolves a common bug that often results in loss of connection
    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "WAITING");
            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
        }

        telemetry.update();
    }
}

