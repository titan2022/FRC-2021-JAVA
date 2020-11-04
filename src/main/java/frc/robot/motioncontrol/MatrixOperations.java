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

    /**
     * Multiplies 2 arrays (order matters and array1.length == array2[i].length)
     * 
     * @param array1 - First 2D double array.
     * @param array2 - Second 2D double array.
     * @return Product of two matrices.
     */
    
    public static double[][] matrixMultiply(double[][] array1, double[][] array2) {

        double[][] productMatrix = new double[array1.length][array2[0].length];

        for (int i = 0; i < array1.length; i++) {

            for (int j = 0; j < array2[0].length; j++) {

                for (int k = 0; k < array1[0].length; k++) {

                    productMatrix[i][j] += array1[i][k] * array2[k][j];

                }

            }

        }

        return productMatrix;

    }

}
