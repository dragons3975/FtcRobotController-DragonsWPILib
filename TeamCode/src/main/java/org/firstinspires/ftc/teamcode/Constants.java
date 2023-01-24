package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class ModesConstants {
        enum Modes {
            auto,
            teleop,
            test
        }
    }

    public static final class DriveConstants {
        public static final double kDiametreRoue = 10;
        public static final double kCmParTour = Math.PI * kDiametreRoue;
        public static final double kTickParTour = 1440;
        public static final double kCmParTick = kCmParTour / kTickParTour;
    }

    public static final class PIDxConstants {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 5;
    }

    public static final class PIDyConstants {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 5;
    }

    public static final class PIDzConstants {
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 5;
    }

    public static final class PIDascenseurConstants {
        public static final double kPMoy = 0.4; //Faire un calcul avec la charge de la batterie
        public static final double kIMoy = 0;
        public static final double kDMoy = 0;
        public static final double kMoyTolerance = 0.8;

        public static final double kPDiff = 0.6;
        public static final double kIDiff = 0;
        public static final double kDDiff = 0;
        public static final double kDiffTolerance = 0.5;
    }

    public static final class AscenseurConstants {
        private static final double kDiametrePoulie = 1.86;
        private static final double kCmParTour = Math.PI * kDiametrePoulie;
        //public static final double kTickParTour = 1440; //Encodeurs des moteurs, ne pas supprimer
        public static final double kTickParTour = -8192; //Encodeurs REV, important de laisser le signe ici pour rester compatible si on remet les encodeurs des moteurs
        private static final int knbEtages = 3;
        public static final double kCmParTick = knbEtages * kCmParTour / kTickParTour;

        public static final double kDefaultDeltaCoeff = 2;

        public static final double kVitesseCalibration = -0.2;

        public static final double kPositionSol = 5;
        public static final double kPositionBas = 35;
        public static final double kPositionMoyen = 59;
        public static final double kPositionHaut = 80;

        public static final double kPositionMin = 0;
        public static final double kPositionMax = 80;
    }

    public static final class PinceConstants {
        public static final double kOuvrirPincePosition = 0.27;
        public static final double kFermerPincePosition = 0;
        public static final double kOuvrirFermerPinceTimeout = 1;
    }

    public static final class VisionConstants {
        public static final double kfx = 822.317;
        public static final double kfy = 822.317;
        public static final double kcx = 319.495;
        public static final double kcy = 242.502;
        public static final double ktagsize = 0.166;
    }

 }
