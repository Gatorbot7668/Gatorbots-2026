// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Map.Entry;
import java.util.TreeMap;

/**
 * An interpolating lookup table for shooter parameters.
 * 
 * Maps distance (meters, calculated from Limelight ty) to ShooterParameters.
 * When querying a value between sample points, it linearly 
 * interpolates all parameters.
 * 
 * <p><b>HOW THE INTERPOLATION CHAIN WORKS:</b>
 * <pre>
 * STEP 1: CANFuelSubsystem calls SHOOTER_LOOKUP_TABLE.get(distance)
 *         where distance = meters to target from VisionSubsystem.getDistanceToTarget()
 *
 * STEP 2: InterpolatingShooterMap.get(2.5) runs:
 *         - Finds floor sample: 2.0m -> ShooterParameters(3500, 3500)
 *         - Finds ceiling sample: 3.0m -> ShooterParameters(4400, 4400)
 *         - Calculates t = (2.5-2.0)/(3.0-2.0) = 0.5 (50% of the way)
 *         - Calls: floor.interpolate(ceiling, 0.5)
 *
 * STEP 3: ShooterParameters.interpolate() runs:
 *         - Calls lerp(3500, 4400, 0.5) for launcherRPM
 *         - Calls lerp(3500, 4400, 0.5) for feederRPM
 *
 * STEP 4: lerp() does the math:
 *         - 3500 + (4400 - 3500) * 0.5 = 3950 RPM
 *
 * STEP 5: Returns ShooterParameters(3950, 3950) back up the chain
 *         CANFuelSubsystem uses this to call setLauncherRPM(3950)
 * </pre>
 * 
 * <p>Usage:
 * <pre>
 * InterpolatingShooterMap map = new InterpolatingShooterMap();
 * 
 * // Add samples from testing (distance_meters, parameters)
 * map.addSample(0.5, new ShooterParameters(2260, 2260));  // very close
 * map.addSample(2.5, new ShooterParameters(3900, 3900));  // medium
 * map.addSample(5.0, new ShooterParameters(6784, 6784));  // very far
 * 
 * // Query - automatically interpolates between points
 * ShooterParameters params = map.get(1.5); // interpolates between 0.5 and 2.5
 * setLauncherRPM(params.launcherRPM);
 * </pre>
 * 
 * <p>IMPORTANT: You need 10-15+ good sample points for smooth interpolation.
 * Collect these through trial and error on the real robot!
 */
public class InterpolatingShooterMap {
    private final TreeMap<Double, ShooterParameters> samples = new TreeMap<>();

    /**
     * Add a sample point to the lookup table.
     * @param key The distance in meters (from VisionSubsystem.getDistanceToTarget())
     * @param parameters The shooter parameters that work at this distance
     */
    public void addSample(double key, ShooterParameters parameters) {
        samples.put(key, parameters);
    }

    /**
     * Get interpolated shooter parameters for the given input.
     * 
     * If the key exactly matches a sample, returns that sample.
     * If the key is between two samples, linearly interpolates.
     * If the key is outside the range, clamps to the nearest sample.
     * 
     * @param key The input value to look up
     * @return Interpolated ShooterParameters
     * @throws IllegalStateException if the map is empty
     */
    public ShooterParameters get(double key) {
        if (samples.isEmpty()) {
            throw new IllegalStateException("InterpolatingShooterMap has no samples!");
        }

        // Exact match
        ShooterParameters exact = samples.get(key);
        if (exact != null) {
            return exact;
        }

        // Get surrounding entries
        Entry<Double, ShooterParameters> floor = samples.floorEntry(key);
        Entry<Double, ShooterParameters> ceiling = samples.ceilingEntry(key);

        // Handle edge cases (outside the sample range)
        if (floor == null) {
            return ceiling.getValue(); // Below minimum - use lowest sample
        }
        if (ceiling == null) {
            return floor.getValue(); // Above maximum - use highest sample
        }

        // ==================================================================
        // INTERPOLATION HAPPENS HERE:
        // 
        // 1. Calculate t = how far between floor and ceiling is our distance?
        //    Example: distance=2.5m, floor=2.0m, ceiling=3.0m
        //             t = (2.5 - 2.0) / (3.0 - 2.0) = 0.5 (50% of the way)
        //
        // 2. Call interpolate() which calls lerp() to blend the RPM values:
        //    launcherRPM = floorRPM + (ceilingRPM - floorRPM) * t
        //    Example: 3500 + (4400 - 3500) * 0.5 = 3950 RPM
        // ==================================================================
        double t = (key - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        
        // This calls ShooterParameters.interpolate() → which calls lerp() → returns blended RPM
        return floor.getValue().interpolate(ceiling.getValue(), t);
    }

    /**
     * @return The number of sample points in the table
     */
    public int size() {
        return samples.size();
    }

    /**
     * @return true if the table has no samples
     */
    public boolean isEmpty() {
        return samples.isEmpty();
    }

    /**
     * @return The minimum key (closest distance)
     */
    public double getMinKey() {
        return samples.firstKey();
    }

    /**
     * @return The maximum key (farthest distance)
     */
    public double getMaxKey() {
        return samples.lastKey();
    }
}
