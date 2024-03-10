package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(BrasSubsystem brasSubsystem) {
        //BrasCommandGroup brasCommandGroup= new BrasCommandGroup(brasSubsystem, Constants.BrasPosRamasseurConstants.posMoteur, Constants.BrasPosRamasseurConstants.posAvantBras, Constants.BrasPosRamasseurConstants.posRotationMain);

        addCommands(
                //brasCommandGroup
                );
    }

}