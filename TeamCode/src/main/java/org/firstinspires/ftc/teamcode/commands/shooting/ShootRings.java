package org.firstinspires.ftc.teamcode.commands.shooting;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.cartridge.PushRing;
import org.firstinspires.ftc.teamcode.commands.cartridge.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRings extends SequentialCommandGroup {
    private final ShooterSubsystem shooter;
    private final CartridgeSubsystem cartridge;
    private final int numRings;
    private final int ringShotDelay;

    public ShootRings(ShooterSubsystem shooter, CartridgeSubsystem cartridge, int numRings, int ringShotDelay, Telemetry telemetry) {
        this.shooter = shooter;
        this.cartridge = cartridge;
        this.numRings = numRings;
        this.ringShotDelay=ringShotDelay;

        addCommands(new WaitCommand(250));

        for (int i = 0; i < numRings; i++) {
            addCommands(new ShootRing(shooter, cartridge, telemetry));

            if (i != numRings - 1)
                addCommands(/*new StartShooter(shooter, telemetry)*/new WaitCommand(ringShotDelay));
        }

        addCommands(new WaitCommand(200));

        addRequirements(shooter, cartridge);
    }
}
