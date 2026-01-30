package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class LanceurSubsystem extends Subsystem {

    private final FtcMotor mMotorLanceur = new FtcMotor("lanceur");
    private int[] mEncPrec = new int[25];
    private final PIDController mPID = new PIDController(Constants.LanceurConstants.kP, 0, 0);

    private double mConsigneDeltaMoy;
    public LanceurSubsystem() {
        mMotorLanceur.setInverted(false);
        mPID.setTolerance(3);
    }

    @Override
    public void periodic() {

        int encAct = mMotorLanceur.getCurrentPosition();
        DriverStationJNI.getTelemetry().addData("Motor lanceur encodeur", encAct);

        for(int i = 0; i <= 23; i++) {
            mEncPrec[i] = mEncPrec[i+1];
        }
        mEncPrec[24] = encAct;

        int[] deltaEnc = new int[24];
        for(int i = 0; i <= 23; i++) {
            deltaEnc[i] = mEncPrec[i+1] - mEncPrec[i];
        }

        int deltaMoy = 0;
        for(int i = 0; i <= 23; i++) {
            deltaMoy = deltaMoy + deltaEnc[i];
        }
        deltaMoy = deltaMoy / 24;

        DriverStationJNI.getTelemetry().addData("DeltaMoy", deltaMoy);

        DriverStationJNI.getTelemetry().addData("vitAct", mMotorLanceur.get());

        double output = mPID.calculate(deltaMoy, mConsigneDeltaMoy);
        DriverStationJNI.getTelemetry().addData("pidOutput", output);
       mMotorLanceur.set(mMotorLanceur.get() + output);

       DriverStationJNI.getTelemetry().addData("IsAtSetSpeed", isAtSetSpeed());

    }


    public void setDeltaMoyConsigne(double deltaMoy) {
        mConsigneDeltaMoy = deltaMoy;
    }

    public boolean isAtSetSpeed(){
        return mPID.atSetpoint();
    }

}