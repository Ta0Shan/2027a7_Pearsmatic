package first.robot.util;

import org.wpilib.command3.Command;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class EnergyTracker {
  public enum Subsystem {
    DRIVE,
    STEER,
    TELESCOPE_PIVOT,
    TELESCOPE_EXTENSION,
    EE_WRIST,
    EE_ROLLERS,
    LAUNCHER,
    UNASSIGNED
  }

  private static double totalChargeConsumedAh = 0.0;
  private static double totalEnergyConsumedWh = 0.0;

  private static Map<Subsystem, Double> subsystemCharges = new HashMap<>();
  private static Map<Subsystem, Double> subsystemEnergies = new HashMap<>();

  public static void reportCurrentUsage(
      double deltaHours, Subsystem subsystem, double... supplyCurrentDrawAmps) {
    double totalAmps = 0.0;
    for (double amp : supplyCurrentDrawAmps) totalAmps += amp;

    if (deltaHours > 0) {
      // 3 600 000 000 microseconds per hour
      double deltaAmpHours = totalAmps * deltaHours;
      double deltaWattHours = deltaAmpHours * RobotController.getBatteryVoltage();

      totalChargeConsumedAh += deltaAmpHours;
      totalEnergyConsumedWh += deltaWattHours;

      subsystemCharges.merge(subsystem, deltaAmpHours, Double::sum);
      subsystemEnergies.merge(subsystem, deltaWattHours, Double::sum);
    }
  }

  public static Command logEnergy() {
    return Command.noRequirements(co -> {
    Telemetry.log("EnergyTracker/Total Charge Amp Hours", totalChargeConsumedAh);
    Telemetry.log("EnergyTracker/Total Energy Watt Hours", totalEnergyConsumedWh);

    for (var entry : subsystemCharges.entrySet()) {
      Telemetry.log(
          "EnergyTracker/Charges/" + entry.getKey().toString() + " Amp Hours", entry.getValue());
      co.yield();
    }

    for (var entry : subsystemEnergies.entrySet()) {
      Telemetry.log(
          "EnergyTracker/Energies/" + entry.getKey().toString() + " Watt Hours", entry.getValue());
      co.yield();
    }
  }).named("ENERGY LOGS");
  }
}
