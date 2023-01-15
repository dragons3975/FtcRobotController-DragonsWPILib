package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class DriveConstants {
        public static final double kInchToMm = 25.4;
        public static final double kCmParTour = Math.PI*10;
        public static final double kTickParTour = 1440;
        public static final double kCmParTick = kCmParTour/kTickParTour;
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
        public static final double kToleranceAscenseur = 5;
        private static final double kDiametrePoulie = 1.86;
        private static final double kCmParTour = Math.PI*kDiametrePoulie;
        private static final double kTickParTour = 1440;
        public static final double kTickParCm = kTickParTour/kCmParTour/3;
        public static final double kCmParTick = 3*kCmParTour/kTickParTour;

        public static final double kDeltaMonterAscenseurCm = 0.2;
        public static final double kDeltaDescendreAscenseurCm = -0.2;

        public static final double kVitesseCalibration = -0.2;

        //1 tour de x cm = 3x cm de hauteur
        public static final double kPositionSol = 5; //Après on fait une calibration pour revenir à 0
        public static final double kPositionBas = 35;
        public static final double kPositionMoyen = 59;
        public static final double kPositionHaut = 80;

        public static final double kPositionMin = 0;
        public static final double kPositionMax = 80;
    }

    public static final class VuforiaConstants {
        public static final String kVuforiaKey = "Enoch";
    }

    public static final class PinceConstants {
        public static final double kOuvrirPince = 0.27;
        public static final double kFermerPince = 0;
    }
}
