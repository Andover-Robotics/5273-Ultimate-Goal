package org.firstinspires.ftc.teamcode.commands.cartridge;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;

public class LowerCartridge extends CommandBase {
    private final CartridgeSubsystem cartridgeSubsystem;

    public LowerCartridge(CartridgeSubsystem cartridgeSubsystem){
        this.cartridgeSubsystem=cartridgeSubsystem;
        addRequirements(cartridgeSubsystem);
    }

    @Override
    public void initialize() {
        cartridgeSubsystem.lowerCartridge();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}