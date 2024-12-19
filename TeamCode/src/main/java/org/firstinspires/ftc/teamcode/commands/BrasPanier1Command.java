package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class BrasPanier1Command extends Command{

    private final BrasSubsystem mBrasSubsystem;
    private final XboxController mxBoxController2;

    public BrasPanier1Command(BrasSubsystem brasSubsystem, XboxController xboxController2) {
        mBrasSubsystem = brasSubsystem;
        mxBoxController2 = xboxController2;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mBrasSubsystem.incrementConsigne(mxBoxController2.getLeftY() * 3);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }
}

