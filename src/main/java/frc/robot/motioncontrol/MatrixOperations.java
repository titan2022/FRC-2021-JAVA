package frc.robot.motioncontrol;

public class MatrixOperations {

    /**
     * Multiplies 2D array by a scalar
     * 
     * @param scalar - Scalar to multiply by.
     * @param array  - Array to be multiplied (class variable).
     * @return Scalar-multiplied array.
     * 
     */

    public static double[][] scalarMultiply(double scalar, double[][] array) {

        array = array.clone();

        for (int i = 0; i < array.length; i++) {

            for (int j = 0; j < array[i].length; j++) {

                array[i][j] *= scalar;

            }

        }

        return array;

    }

    /**
     * Transposes a 2D double array
     * 
     * @param array
     * @return Transposed array.
     */

    public static double[][] transpose(double[][] array) {

        double[][] transposed = new double[array[0].length][array.length];

        for (int i = 0; i < array.length; i++) {

            for (int j = 0; j < array[0].length; j++) {

                transposed[j][i] = array[i][j];

            }

        }

        return transposed;

    }

}
