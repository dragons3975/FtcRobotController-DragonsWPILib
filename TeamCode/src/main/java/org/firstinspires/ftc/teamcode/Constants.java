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
        public static final double kP = 0.10;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 5;
    }

    public static final class AscenseurConstants {
        public static final double kToleranceAscenseur = 5;
        private static final double kDiametrePoulie = 1.86;
        private static final double kCmParTour = Math.PI*kDiametrePoulie;
        private static final double kTickParTour = 1440;
        public static final double kTickParCm = kTickParTour/kCmParTour;
        public static final double kCmParTick = kCmParTour/kTickParTour;


        //1 tour de x cm = 3x cm de hauteur

        public static final double kPositionSol = 10;
        public static final double kPositionBas = 50;
        public static final double kPositionMoyen = 75;
        public static final double kPositionHaut = 100;
    }

    public static final class VuforiaConstants {
        public static final String kVuforiaKey = "Enoch";
    }
}
