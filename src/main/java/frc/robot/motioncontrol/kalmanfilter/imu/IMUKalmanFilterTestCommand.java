package frc.robot.motioncontrol.kalmanfilter.imu;

import frc.robot.motioncontrol.kalmanfilter.KalmanFilter;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

import static com.github.swrirobotics.bags.reader.BagReader.readFile;
import com.github.swrirobotics.bags.reader.BagFile;
import com.github.swrirobotics.bags.reader.MessageHandler;
import com.github.swrirobotics.bags.reader.exceptions.BagReaderException;
import com.github.swrirobotics.bags.reader.exceptions.UninitializedFieldException;
import com.github.swrirobotics.bags.reader.messages.serialization.MessageType;
import com.github.swrirobotics.bags.reader.messages.serialization.StringType;
import com.github.swrirobotics.bags.reader.records.Connection;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IMUKalmanFilterTestCommand extends CommandBase {

    private KalmanFilter filter;
    private BagFile bag;
    private ArrayList<String> messages;

    public IMUKalmanFilterTestCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        messages = new ArrayList<String>();

        // from https://github.com/swri-robotics/bag-reader-java#print-all-of-the-std_msgstring-values-in-a-bag

        MessageHandler handler = new MessageHandler(){
        
            @Override
            public boolean process(MessageType message, Connection connection) {

                try {

                    messages.add(message.<StringType>getField("data").getValue());

                } catch (UninitializedFieldException exception) {

                    System.err.println(exception.toString());

                }

                return true;

            }
        };

        try {

            bag = readFile("IMUAccelGyroData.bag");
            bag.forMessagesOfType("std_msgs/String", handler);

        } catch (BagReaderException exception) {

            System.err.println(exception.toString());

        }

    }

}