package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.ActivateAprilTagPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.ActivatePropPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.DeactivatePropPipelineCommand;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ActivateAprilSequentialCommandGroup extends SequentialCommandGroup {

    public ActivateAprilSequentialCommandGroup(VisionSubsystem visionSubsystem) {
        ActivatePropPipelineCommand activateProp = new ActivatePropPipelineCommand(visionSubsystem);
        DeactivatePropPipelineCommand desactivateProp = new DeactivatePropPipelineCommand(visionSubsystem);
        ActivateAprilTagPipelineCommand activateApril = new ActivateAprilTagPipelineCommand(visionSubsystem);

        WaitCommand wait = new WaitCommand(5.0);
        WaitCommand wait2 = new WaitCommand(5.0);

        addCommands(
           activateProp,
           desactivateProp,
           wait2,
           activateApril
        );
    }

}