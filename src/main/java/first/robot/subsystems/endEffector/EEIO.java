package first.robot.subsystems.endEffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import first.robot.Constants.CrystalColor;
import first.robot.util.PearadoxTalonFX.MotorData;

public interface EEIO {
    @AutoLog
    public static class EEIOInputs {
        public MotorData wristData;
        public MotorData rollerData;

        public CrystalColor colorReading = CrystalColor.NONE;
    }

    public default void updateInputs(EEIOInputs inputs) {}

    public default void setWristAngleDeg(double angleDeg) {}

    public default void setWristAngleDeg(double angleDeg, DoubleSupplier ff) {}

    public default void setRollerVoltage(double volts) {}

    public default CrystalColor getColorReading() {return CrystalColor.NONE;}
}
