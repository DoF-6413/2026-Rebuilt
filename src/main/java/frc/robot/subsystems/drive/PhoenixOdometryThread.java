// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock m_signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] m_phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> m_genericSignals = new ArrayList<>();
  private final List<Queue<Double>> m_phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> m_genericQueues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

  private static boolean isCANFD = TunerConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (m_timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    m_signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[m_phoenixSignals.length + 1];
      System.arraycopy(m_phoenixSignals, 0, newSignals, 0, m_phoenixSignals.length);
      newSignals[m_phoenixSignals.length] = signal;
      m_phoenixSignals = newSignals;
      m_phoenixQueues.add(queue);
    } finally {
      m_signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    m_signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      m_genericSignals.add(signal);
      m_genericQueues.add(queue);
    } finally {
      m_signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      m_timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      m_signalsLock.lock();
      try {
        if (isCANFD && m_phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, m_phoenixSignals);
        } else {
          // "waitForAll" does not support blocking on multiple signals with a bus
          // that is not CAN FD, regardless of Pro licensing. No reasoning for this
          // behavior is provided by the documentation.
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (m_phoenixSignals.length > 0) BaseStatusSignal.refreshAll(m_phoenixSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        m_signalsLock.unlock();
      }

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        // Default timestamps from Phoenix are NOT compatible with
        // FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : m_phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (m_phoenixSignals.length > 0) {
          timestamp -= totalLatency / m_phoenixSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < m_phoenixSignals.length; i++) {
          m_phoenixQueues.get(i).offer(m_phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < m_genericSignals.size(); i++) {
          m_genericQueues.get(i).offer(m_genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < m_timestampQueues.size(); i++) {
          m_timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
