package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class PinceSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotor mMoteurPince;
    private final PIDController mPIDpince = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);


    private double mGetAscenseur1;



    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public PinceSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurPince = mHardwareMap.get(DcMotor.class, "Moteur pince");//************ À CHANGER***********************************

        mMoteurPince.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurPince.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        mGetAscenseur1 = mMoteurPince.getCurrentPosition();
        mPIDpince.setTolerance(5);

       // mSetAscenseur = mMoteurAscenseur1.setTargetPosition();
//setpositionAscenseur = mMoteurAscenseur1.setposition et mMoteurAScenseur2.setposition (THÉORIQUE)
    }


    @Override
    public void periodic() {

    }

    public void setSetPointPince(double consigne) {
        mPIDpince.setSetpoint(consigne);
    }

    public boolean atSetPointPince() {
        return mPIDpince.atSetpoint();
    }

    public void stop() {
        mMoteurPince.setPower(0);
    }

}

