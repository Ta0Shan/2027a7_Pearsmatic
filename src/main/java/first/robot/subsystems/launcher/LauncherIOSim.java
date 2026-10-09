package first.robot.subsystems.launcher;

import org.wpilib.math.util.Units;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;
import first.robot.subsystems.launcher.LauncherConstants.LauncherInputs;

public class LauncherIOSim extends LauncherIOTalonFX {
    private final FlywheelSim launcher1PhysicsSim = new FlywheelSim(Models.flywheelFromPhysicalConstants(
        DCMotor.getKrakenX60Foc(1),
        0.5 * (Units.lbsToKilograms((0.6))) * (Math.pow(Units.inchesToMeters(2), 2)), 
        LauncherConstants.REDUCTION),
        DCMotor.getKrakenX60Foc(1)
    );
    
    // mirrored copy but since they have to go at different speeds they're separate
    private final FlywheelSim launcher2PhysicsSim = new FlywheelSim(Models.flywheelFromPhysicalConstants(
        DCMotor.getKrakenX60Foc(1),
        0.5 * (Units.lbsToKilograms((0.6))) * (Math.pow(Units.inchesToMeters(2), 2)), 
        LauncherConstants.REDUCTION),
        DCMotor.getKrakenX60Foc(1)
    );

    private final TalonFXSimState launcher1SimState;
    private final TalonFXSimState launcher2SimState;

    private double launcher1Position = 0.0;
    private double launcher2Position = 0.0;

    public LauncherIOSim() {
        super();
        launcher1SimState = launcher1.getSimState();
        launcher2SimState = launcher2.getSimState();
    }

    @Override
    public LauncherInputs updateInputs() {
        launcher1SimState.setSupplyVoltage(12);
        launcher1PhysicsSim.setInputVoltage(launcher1SimState.getMotorVoltage());
        launcher2SimState.setSupplyVoltage(12);
        launcher2PhysicsSim.setInputVoltage(launcher2SimState.getMotorVoltage());

        launcher1PhysicsSim.update(Constants.LOOP_PERIOD_SEC);
        launcher2PhysicsSim.update(Constants.LOOP_PERIOD_SEC);

        launcher1Position += launcher1PhysicsSim.getAngularVelocity() * Constants.LOOP_PERIOD_SEC * LauncherConstants.REDUCTION;
        launcher2Position += launcher2PhysicsSim.getAngularVelocity() * Constants.LOOP_PERIOD_SEC * LauncherConstants.REDUCTION;

        launcher1SimState.setRawRotorPosition(Units.radiansToRotations(launcher1Position));
        launcher1SimState.setRotorVelocity(Units.radiansToRotations(launcher1PhysicsSim.getAngularVelocity()) * LauncherConstants.REDUCTION);
        launcher2SimState.setRawRotorPosition(Units.radiansToRotations(launcher2Position));
        launcher2SimState.setRotorVelocity(Units.radiansToRotations(launcher2PhysicsSim.getAngularVelocity()) * LauncherConstants.REDUCTION);

        return super.updateInputs();
    }
}
