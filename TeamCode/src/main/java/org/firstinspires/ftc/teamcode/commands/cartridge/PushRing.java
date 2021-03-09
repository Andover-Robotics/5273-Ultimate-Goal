package org.firstinspires.ftc.teamcode.commands.cartridge;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;

public class PushRing extends CommandBase {
    private CartridgeSubsystem cartridge;

    public PushRing(CartridgeSubsystem cartridge) {
        this.cartridge = cartridge;

        addRequirements(cartridge);
    }

    @Override
    public void initialize() {
        cartridge.pushArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
