package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceXAutoCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionEchangeExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMaxPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Gauche extends SequentialCommandGroup {

    public Gauche(DriveSubsystem driveSubsystem, BrasSubsystem brasSubsystem, PinceBrasSubsystem pinceBrasSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem, ExtensionSubsystem extensionSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        ParallelRaceGroup avance = new AvanceXAutoCommand(driveSubsystem,15, -60).withTimeout(2);
        ParallelRaceGroup tourne = new TourneAutoCommand(driveSubsystem, 45).withTimeout(2);
        PanierAutoCommand panier = new PanierAutoCommand(brasSubsystem, pinceBrasSubsystem, pinceExtensionSubsystem);
        PositionMaxPinceBrasCommand pinceMax = new PositionMaxPinceBrasCommand(pinceBrasSubsystem);
        OpenPinceBrasCommand open = new OpenPinceBrasCommand(pinceBrasSubsystem);
        PositionMinPinceBrasCommand pinceMin = new PositionMinPinceBrasCommand(pinceBrasSubsystem);
        BrasPositionCommandtest descendBras = new BrasPositionCommandtest(brasSubsystem, -300, 0.5);
        ParallelRaceGroup tourne2 = new TourneAutoCommand(driveSubsystem, 0).withTimeout(2);
        ParallelRaceGroup avance2 = new AvanceXAutoCommand(driveSubsystem, 57, -48).withTimeout(2.5);
        //ParallelRaceGroup avance3 = new AvanceXAutoCommand(driveSubsystem, 62, -67).withTimeout(1);
        ExtendPositionCommand extendre = new ExtendPositionCommand(extensionSubsystem, 2300);
        PinceRotationCommand rotationZero = new PinceRotationCommand(pinceExtensionSubsystem, 0.55);
        PincePositionEchangeExtCommand rotationSol = new PincePositionEchangeExtCommand(pinceExtensionSubsystem, 0.05);
        ClosePinceExtCommand close = new ClosePinceExtCommand(pinceExtensionSubsystem);
        RamasseurCommand ramasseurCommand = new RamasseurCommand(extensionSubsystem, pinceExtensionSubsystem, pinceBrasSubsystem, brasSubsystem);
        ParallelRaceGroup retournePanier = new AvanceXAutoCommand(driveSubsystem,12, -48).withTimeout(2);
        ParallelRaceGroup tourne3 = new TourneAutoCommand(driveSubsystem, 45).withTimeout(2);
        PanierAutoCommand panier2 = new PanierAutoCommand(brasSubsystem, pinceBrasSubsystem, pinceExtensionSubsystem);
        PositionMaxPinceBrasCommand pinceMax2 = new PositionMaxPinceBrasCommand(pinceBrasSubsystem);
        OpenPinceBrasCommand open2 = new OpenPinceBrasCommand(pinceBrasSubsystem);
        PositionMinPinceBrasCommand pinceMin2 = new PositionMinPinceBrasCommand(pinceBrasSubsystem);
        BrasPositionCommandtest descendBras2 = new BrasPositionCommandtest(brasSubsystem, -300, 0.5);
        ParallelRaceGroup tourne4 = new TourneAutoCommand(driveSubsystem, 0).withTimeout(2);
        ParallelRaceGroup avance3 = new AvanceXAutoCommand(driveSubsystem,56, -25).withTimeout(3);

        addCommands(
                avance,
                //tasse,
                tourne,
                panier,
                //recule
                pinceMax,
                new WaitCommand(0.5),
                open,
                new WaitCommand(0.2),
                pinceMin,
                new WaitCommand(0.2),
                descendBras,
                tourne2,
                avance2,
                rotationZero,
                extendre,
                new WaitCommand(1),
                rotationSol,
                new WaitCommand(1),
                close,
                new WaitCommand(0.3),
                ramasseurCommand,
                retournePanier,
                tourne3,
                panier2,
                pinceMax2,
                new WaitCommand(0.5),
                open2,
                new WaitCommand(0.2),
                pinceMin2,
                new WaitCommand(0.2),
                descendBras2,
                tourne4,
                avance3

        );
    }

}