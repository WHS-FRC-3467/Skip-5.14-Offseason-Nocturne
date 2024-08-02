package frc.robot.Util;

import java.util.Collections;
import java.util.List;

/**
 * Lookup table to get the correct shooter preset for the given distance from the target
 *
 * Inspired by Team 4481 - Team Rembrandts who were in turn inspired by Team 3476 - Code Orange
 */
public class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }

    public VisionLookUpTable() {
        shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(-12, 30, 30, 1.0)); //100 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(-5, 35, 35, 1.5)); //150 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 40, 40, 2.0)); //200 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(4, 50, 50, 2.5)); //250 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(7, 60, 50, 3.0)); //300 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(10, 70, 55, 3.5)); //350 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(12, 75, 60, 4.0)); //400 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(14, 75, 60, 4.5)); //450 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(16, 75, 65, 5.0)); //500 CM -> BUMPER
        shooterConfig.getShooterConfigs().add(new ShooterPreset(18, 75, 65, 5.5)); //550 CM -> BUMPER

        // Sort the list from smallest to largest distance
        Collections.sort(shooterConfig.getShooterConfigs());
    }

    /**
     * Obtains a shooter preset from a given target distance
     *
     * @param pDistanceFromTarget measured distance to the shooting target
     * @return new shooter preset for given distance
     */
    public ShooterPreset getShooterPreset(double pDistanceFromTarget){
        int endIndex = shooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance.
         */
        if(pDistanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()){
            return shooterConfig.getShooterConfigs().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance.
         */
        if(pDistanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return shooterConfig.getShooterConfigs().get(endIndex);
        }

        /*
         * If the measured distance falls somewhere within the lookup table perform a binary search within the lookup
         * table
         */
        return binarySearchDistance(shooterConfig.getShooterConfigs(),0, endIndex, pDistanceFromTarget);
    }

    /**
     * Perform fast binary search to find a matching shooter preset. If no matching preset is found it interpolates a
     * new shooter preset based on the two surrounding table entries.
     *
     * @param pShooterConfigs The table containing the shooter presets
     * @param pStartIndex Starting point to search
     * @param pEndIndex Ending point to search
     * @param pDistance Distance for which we need to find a preset
     *
     * @return (Interpolated) shooting preset
     */
    private ShooterPreset binarySearchDistance(List<ShooterPreset> pShooterConfigs,int pStartIndex, int pEndIndex, double pDistance)
    {
        int mid = pStartIndex + (pEndIndex - pStartIndex) / 2;
        double midIndexDistance = pShooterConfigs.get(mid).getDistance();

        // If the element is present at the middle
        // return itself
        if (pDistance == midIndexDistance)
            return pShooterConfigs.get(mid);

        // If only two elements are left
        // return the interpolated config
        if (pEndIndex - pStartIndex == 1) {
            double percentIn = (pDistance - shooterConfig.getShooterConfigs().get(pStartIndex).getDistance()) /
                    (
                            shooterConfig.getShooterConfigs().get(pEndIndex).getDistance() -
                                    shooterConfig.getShooterConfigs().get(pStartIndex).getDistance()
                    );
            return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(pStartIndex), shooterConfig.getShooterConfigs().get(pEndIndex), percentIn);
        }

        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (pDistance < midIndexDistance)
            return binarySearchDistance(pShooterConfigs, pStartIndex, mid, pDistance);


        // Else the element can only be present
        // in right subarray
        return binarySearchDistance(pShooterConfigs, mid, pEndIndex, pDistance);
    }

    /**
     * Obtain a new shooter preset by interpolating between two existing shooter presets.
     *
     * @param pStartPreset Starting preset for interpolation
     * @param pEndPreset Ending preset for interpolation
     * @param pPercentIn Amount of percentage between the two values the new preset needs to be
     *
     * @return new interpolated shooter preset
     */
    private ShooterPreset interpolateShooterPreset(ShooterPreset pStartPreset, ShooterPreset pEndPreset, double pPercentIn) {
        double leftShooterSpeed = pStartPreset.getLeftShooterSpeed() + (pEndPreset.getLeftShooterSpeed() - pStartPreset.getLeftShooterSpeed()) * pPercentIn;
        double rightShooterSpeed = pStartPreset.getRightShooterSpeed() + (pEndPreset.getRightShooterSpeed() - pStartPreset.getRightShooterSpeed()) * pPercentIn;
        double armPosition = pStartPreset.getArmAngle() + (pEndPreset.getArmAngle() - pStartPreset.getArmAngle()) * pPercentIn;
        double distance = pStartPreset.getDistance() + (pEndPreset.getDistance() - pStartPreset.getDistance()) * pPercentIn;

        return new ShooterPreset(armPosition, leftShooterSpeed, rightShooterSpeed, distance);
    }

    /**
     * <b>MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION</b>
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig pShooterConfig) {
        this.shooterConfig = pShooterConfig;
    }
}