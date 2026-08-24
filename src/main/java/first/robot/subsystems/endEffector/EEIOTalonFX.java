package first.robot.subsystems.endEffector;

import org.wpilib.math.util.Units;
import org.wpilib.util.Color;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import first.robot.Constants;
import first.robot.util.PearadoxTalonFX;
import first.robot.util.EnergyTracker.Subsystem;

public abstract class EEIOTalonFX implements EEIO {
    protected final PearadoxTalonFX wrist;
    protected final PearadoxTalonFX roller;

    protected final PositionVoltage positionVoltage;
    protected final VoltageOut voltageOut;

    public EEIOTalonFX() {
        wrist = new PearadoxTalonFX(EEConstants.WRIST_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            EEConstants.WRIST_CONFIG(),
            Subsystem.EE_WRIST);
        roller = new PearadoxTalonFX(EEConstants.ROLLERS_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            EEConstants.ROLLER_CONFIG(),
            Subsystem.EE_ROLLERS);

        positionVoltage = new PositionVoltage(0);
        voltageOut = new VoltageOut(0);
    }

    public void updateInputs(EEIOInputs inputs) {
        inputs.wristData = wrist.getData();
        inputs.rollerData = roller.getData();

        // inputs.colorReading = getColorReading();
    }

    public void setWristAngleDeg(double angleDeg) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * EEConstants.WRIST_REDUCTION;
        wrist.setControl(positionVoltage.withPosition(motorSetpoint));
    }
    
    public void setWristAngleDeg(double angleDeg, double ff) {
        double motorSetpoint = Units.degreesToRotations(angleDeg) * EEConstants.WRIST_REDUCTION;
        wrist.setControl(positionVoltage.withPosition(motorSetpoint).withFeedForward(ff));
    }

    public void setRollerVoltage(double voltage) {
        roller.setControl(voltageOut.withOutput(voltage));
    }
}
