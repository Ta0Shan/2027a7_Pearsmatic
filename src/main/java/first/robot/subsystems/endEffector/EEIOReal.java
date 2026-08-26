package first.robot.subsystems.endEffector;

import org.wpilib.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import first.robot.Constants.CrystalColor;

public class EEIOReal extends EEIOTalonFX {
    // because CTRE is so nice, they make sim states, which means to simulate we only need to build on the
    // base type.
    // I can't say the same about REV though

    private final ColorSensorV3 colorSensor;

    public EEIOReal() {
        super();
        colorSensor = new ColorSensorV3(EEConstants.COLOR_SENSOR_PORT);
        colorSensor.configureColorSensor(
            ColorSensorResolution.kColorSensorRes20bit,
            ColorSensorMeasurementRate.kColorRate25ms,
            GainFactor.kGain3x);
    }

    @Override
    public void updateInputs(EEIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.colorReading = getColorReading();
    }

    public CrystalColor getColorReading() {
        Color reading = colorSensor.getColor();
        Color output = Color.BLACK;
        double minimumWeightDist = getWeightedDistance(reading, output);
        for (Color color : EEConstants.CRYSTAL_COLORS) {
            double weightedDist = getWeightedDistance(reading, color);
            if (weightedDist < minimumWeightDist) {
                output = color;
                minimumWeightDist = weightedDist;
            }
        }
        return CrystalColor.getFromHex(output.toHexString());
    }

    // cc GEMINI
    /**
     * Calculates a perceptually weighted distance between two colors.
     * Faster than CIELAB conversion, more accurate than raw Euclidean distance.
     */
    private static double getWeightedDistance(Color c1, Color c2) {
        long rmean = ((long) c1.red + (long) c2.blue) / 2;
        long r = (long) c1.red - (long) c2.red;
        long g = (long) c1.green - (long) c2.green;
        long b = (long) c1.blue - (long) c2.blue;
        
        // Squaring the differences and applying the Redmean weights
        long weightR = 2 + (rmean / 256);
        long weightG = 4;
        long weightB = 2 + ((255 - rmean) / 256);
        
        // We omit the Math.sqrt() because comparing squared values yields the same closest color
        // and saves CPU cycles. Remove the comment below if you need the actual distance value.
        return (weightR * r * r) + (weightG * g * g) + (weightB * b * b);
    }
}
