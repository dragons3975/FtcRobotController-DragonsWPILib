package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

public class TestCommand extends Command {

    private final TestSubsystem mTestSubsystem;

    public TestCommand(TestSubsystem testSubsystem) {
        mTestSubsystem = testSubsystem;
        addRequirements(testSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mTestSubsystem.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mTestSubsystem.stop();

    }

}

