package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.AvancerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AvancerAutoCommand extends SequentialCommandGroup {

    public DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    public AvancerAutoCommand(DriveSubsystem driveSubsystem) {

        AvancerCommand avancer20cm = new AvancerCommand(mDriveSubsystem, 1, 0, 0, 20);

        addCommands(
                avancer20cm
        );
    }

}
