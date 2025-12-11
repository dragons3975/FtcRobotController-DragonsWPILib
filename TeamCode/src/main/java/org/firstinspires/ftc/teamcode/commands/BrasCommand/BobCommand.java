package org.firstinspires.ftc.teamcode.commands.BrasCommand;

import org.firstinspires.ftc.teamcode.subsystems.TestRecruesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class BobCommand extends Command {

    private final TestRecruesSubsystem mTestRecruesSubsystem;
    private final TestSubsystem mTestSubsystem;

    public BobCommand(TestRecruesSubsystem testRecruesSubsystem, TestSubsystem testSubsystem) {
        mTestRecruesSubsystem = testRecruesSubsystem;
        mTestSubsystem = testSubsystem;
        addRequirements(testRecruesSubsystem, testSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mTestRecruesSubsystem.pain();
       // mTestSubsystem.pain();
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
        mTestSubsystem.stop();
    }

}
