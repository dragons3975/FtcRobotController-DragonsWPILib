package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class LanceurSubsystem extends Subsystem {

    private final FtcMotor mMotorLanceur = new FtcMotor("lanceur");
    private int[] mEncPrec = new int[10];
    private final PIDController mPID = new PIDController(Constants.LanceurConstants.kP, 0, 0);

    private double mConsigneDeltaMoy;
    public LanceurSubsystem() {
        mMotorLanceur.setInverted(false);
    }

    @Override
    public void periodic() {

        int encAct = mMotorLanceur.getCurrentPosition();
        DriverStationJNI.getTelemetry().addData("Motor lanceur encodeur", encAct);

        for(int i = 0; i <= 8; i++) {
            mEncPrec[i] = mEncPrec[i+1];
        }
        mEncPrec[9] = encAct;

        int[] deltaEnc = new int[9];
        for(int i = 0; i <= 8; i++) {
            deltaEnc[i] = mEncPrec[i+1] - mEncPrec[i];
        }

        int deltaMoy = 0;
        for(int i = 0; i <= 8; i++) {
            deltaMoy = deltaMoy + deltaEnc[i];
        }
        deltaMoy = deltaMoy / 9;

        DriverStationJNI.getTelemetry().addData("DeltaMoy", deltaMoy);

        DriverStationJNI.getTelemetry().addData("vitAct", mMotorLanceur.get());

        double output = mPID.calculate(deltaMoy, mConsigneDeltaMoy);
        DriverStationJNI.getTelemetry().addData("pidOutput", output);
       mMotorLanceur.set(mMotorLanceur.get() + output);

       DriverStationJNI.getTelemetry().addData("IsAtSetSpeed", isAtSetSpeed());

    }


    public void setDeltaMoyConsigne(double deltaMoy) {


        //mConsigneDeltaMoy = deltaMoy;


        //Temporaire pour test
        mConsigneDeltaMoy = deltaMoy;
    }

    public void stop() {
        mConsigneDeltaMoy = 0;
    }
    public boolean isAtSetSpeed(){
        return mPID.atSetpoint();
    }

}