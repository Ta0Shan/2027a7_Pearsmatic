package first.robot.subsystems.endEffector;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;

public class EEIOSim extends EEIOTalonFX {
    private final SingleJointedArmSim wristPhysicsSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(1), 
        EEConstants.WRIST_REDUCTION, 
        SingleJointedArmSim.estimateMOI(EEConstants.LENGTH_METERS, EEConstants.MASS_KG),
        EEConstants.LENGTH_METERS,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY, 
        false, 
        0
    );

    private final FlywheelSim rollerPhysicsSim = new FlywheelSim(
        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60Foc(1),
            (0.5) * (Units.lbsToKilograms(1)) * (Math.pow(Units.inchesToMeters(1), 2)),
            EEConstants.ROLLER_REDUCTION),
        DCMotor.getKrakenX60Foc(1)
    );

    private double rollerPosition;

    private final TalonFXSimState wristSimState;
    private final TalonFXSimState rollerSimState;

    public EEIOSim() {
        super();
        wristSimState = wrist.getSimState();
        rollerSimState = roller.getSimState();
    }

    @Override
    public void updateInputs(EEIOInputs inputs) {
        super.updateInputs(inputs);

        wristSimState.setSupplyVoltage(12);
        wristPhysicsSim.setInputVoltage(wristSimState.getMotorVoltage());

        rollerSimState.setSupplyVoltage(12);
        rollerPhysicsSim.setInputVoltage(rollerSimState.getMotorVoltage());


        wristPhysicsSim.update(Constants.LOOP_PERIOD_SEC);
        rollerPhysicsSim.update(Constants.LOOP_PERIOD_SEC);
        
        
        wristSimState.setRawRotorPosition(Units.radiansToRotations(wristPhysicsSim.getAngle()) * EEConstants.WRIST_REDUCTION);
        wristSimState.setRotorVelocity(Units.radiansToRotations(wristPhysicsSim.getVelocity()) * EEConstants.WRIST_REDUCTION);
        
        rollerPosition += rollerPhysicsSim.getAngularVelocity() * Constants.LOOP_PERIOD_SEC * EEConstants.ROLLER_REDUCTION;
        rollerSimState.setRawRotorPosition(Units.radiansToRotations(rollerPosition));
        rollerSimState.setRotorVelocity(Units.radiansToRotations(rollerPhysicsSim.getAngularVelocity()) * EEConstants.ROLLER_REDUCTION);
    }
}
