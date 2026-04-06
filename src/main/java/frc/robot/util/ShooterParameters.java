// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Holds shooter parameters for a given distance.
 * Used with InterpolatingShooterMap for distance-based shooting.
 * 
 * All values are interpolatable - when looking up a distance between
 * two sample points, each parameter is linearly interpolated.
 */
public class ShooterParameters {
    /** Launcher (flywheel) RPM */
    public final double launcherRPM;
    
    /** Feeder RPM */
    public final double feederRPM;

    /**
     * Create shooter parameters with the same RPM for both motors.
     * @param rpm RPM for both launcher and feeder
     */
    public ShooterParameters(double rpm) {
        this.launcherRPM = rpm;
        this.feederRPM = rpm;
    }

    /**
     * Create shooter parameters with different RPMs for each motor.
     * @param launcherRPM RPM for the launcher (flywheel)
     * @param feederRPM RPM for the feeder
     */
    public ShooterParameters(double launcherRPM, double feederRPM) {
        this.launcherRPM = launcherRPM;
        this.feederRPM = feederRPM;
    }

    /**
     * Linearly interpolate between two ShooterParameters.
     * 
     * Called automatically by InterpolatingShooterMap when ta falls between two samples.
     * 
     * Example:
     *   Sample 1: ta=2.0 → 5500 RPM (this)
     *   Sample 2: ta=4.0 → 4400 RPM (other)
     *   Robot sees ta=3.0 → t=0.5 (halfway)
     *   Result: 4950 RPM (halfway between 5500 and 4400)
     * 
     * @param other The other parameters to interpolate towards (the "ceiling" sample)
     * @param t Interpolation factor calculated by InterpolatingShooterMap:
     *          t = (current_ta - floor_ta) / (ceiling_ta - floor_ta)
     *          t=0.0 means use THIS sample's values
     *          t=1.0 means use OTHER sample's values
     *          t=0.5 means use values halfway between
     * @return New ShooterParameters with interpolated values
     */
    public ShooterParameters interpolate(ShooterParameters other, double t) {
        // Interpolate each motor's RPM separately
        // This allows launcher and feeder to have different interpolation if needed
        return new ShooterParameters(
            lerp(this.launcherRPM, other.launcherRPM, t),  // Interpolate launcher RPM
            lerp(this.feederRPM, other.feederRPM, t)       // Interpolate feeder RPM
        );
    }

    /**
     * Linear interpolation (lerp) - calculates a value between two numbers.
     * 
     * Formula: result = a + (b - a) * t
     * 
     * Examples:
     *   lerp(1000, 2000, 0.0) = 1000  (t=0 returns a)
     *   lerp(1000, 2000, 1.0) = 2000  (t=1 returns b)
     *   lerp(1000, 2000, 0.5) = 1500  (t=0.5 returns halfway)
     *   lerp(1000, 2000, 0.25) = 1250 (t=0.25 returns 25% of the way)
     * 
     * @param a Starting value (floor sample's RPM)
     * @param b Ending value (ceiling sample's RPM)
     * @param t Percentage of the way from a to b (0.0 to 1.0)
     * @return The interpolated value
     */
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /**
     * Returns a readable string for debugging.
     * Example output: "ShooterParameters(launcher=5000.0 RPM, feeder=5000.0 RPM)"
     */
    @Override
    public String toString() {
        return String.format("ShooterParameters(launcher=%.1f RPM, feeder=%.1f RPM)", 
            launcherRPM, feederRPM);
    }
}
