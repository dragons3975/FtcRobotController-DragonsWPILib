package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class ConfigConstants {
        public static final int kRouge = 0;
        public static final int kBleu = 1;
        public static final int kGauche = 0;
        public static final int kDroite = 1;

    }


    public static final class AutonomousConstants {

        //POSITION 1, AVANCEMENT INITIAL
        public static final int kPos1AvanceGauche = 10;//
        public static final int kPos1AvanceMilieu = 20;
        public static final int kPos1AvanceDroite = 30;
        //POSITION 1, APPROCHE AU TABLEAU


        public static final int kApproche = 20;
        //POSITION 2, AVANCEMENT INITIAL

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;
    }

    public static final class ConstantsDrivePID {
        public static final double kP = 0.08;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ConstantsBras {
        public static final double coeficiant = 0.01;

        public static final double kmax = 970;

        public static final double kmaxExt = 47200;

    }

    public static final class ConstantsPince {

        public static final double InclinaisonHaut = 0.75;
        public static final double InclinaisonBas = 0.4;
        public static final double ouvreMax = 0.4;

        public static final double ouvreMin = 0.2;
    }

    public static final class ConstantsDrive {
        public static final double tacho1Tour = 8192.0 / 3.0;

        public static final double diametre = 10;

        public static final double distanceCalcul = (ConstantsDrive.tacho1Tour/(Math.PI * Constants.ConstantsDrive.diametre));


    }

    public static final class VisionConstants {
        public static final String kTfodModelFile = "/sdcard/FIRST/tflitemodels/red_and_blue_camera_robot.tflite";
        public static final int kWidth = 320;
        public static final int kHeight = 240;
        public static final int kLimiteG = 250;

        public static final int kLimiteD = 650;

        public static final int kTeamPropGauche = 0;
        public static final int kTeamPropMilieu = 1;
        public static final int kTeamPropDroite = 2;

    }


    
}
