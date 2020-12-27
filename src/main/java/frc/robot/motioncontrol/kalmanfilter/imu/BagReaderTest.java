package frc.robot.motioncontrol.kalmanfilter.imu;

import com.github.swrirobotics.bags.reader.BagReader;
import com.github.swrirobotics.bags.reader.exceptions.BagReaderException;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.github.swrirobotics.bags.reader.BagFile;

public class BagReaderTest extends CommandBase {

    public BagReaderTest() {

        addRequirements();

    }

    @Override
    public void initialize() {

        try {

            BagFile bag = BagReader.readFile("IMUAccelGyroData.bag");

        } catch (BagReaderException e) {
        }

    }

}