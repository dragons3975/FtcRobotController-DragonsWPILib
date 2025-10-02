package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.TestRecruesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SushiCommand extends Command {

    private final TestRecruesSubsystem mTestRecruesSubsystem;

    public SushiCommand(TestRecruesSubsystem testRecruesSubsystem) {
        mTestRecruesSubsystem = testRecruesSubsystem;
        addRequirements(testRecruesSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mTestRecruesSubsystem.start();
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
        mTestRecruesSubsystem.stop();
    }

}
