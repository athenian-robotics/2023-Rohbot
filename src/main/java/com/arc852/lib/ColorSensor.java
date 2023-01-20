package com.arc852.lib;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.wpilibj.Timer;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

// TODO: make it so you can have many sensors
// TODO: obtain lock before try/catch
public class ColorSensor implements AutoCloseable {
  final Sinks.Many<BallColor> sink;
  private final AtomicBoolean debugPrints = new AtomicBoolean();
  private final RawColor color0 = new RawColor();
  private final ReentrantLock threadLock = new ReentrantLock();
  private final Thread readThread;
  private final AtomicBoolean threadRunning = new AtomicBoolean(true);
  private boolean hasColor0;
  private int prox0;
  private double lastReadTime;

  public ColorSensor() {
    this.sink = Sinks.many().multicast().onBackpressureBuffer();
    readThread = new Thread(this::threadMain);
    readThread.setName("PicoColorSensorThread");
    readThread.start();
  }

  int parseIntFromIndex(SingleCharSequence charSeq, int readLen, IntRef lastComma) {
    int nextComma;
    try {
      nextComma = findNextComma(charSeq.data, readLen, lastComma.value);
      int value = Integer.parseInt(charSeq, lastComma.value + 1, nextComma, 10);
      lastComma.value = nextComma;
      return value;
    } catch (Exception ex) {
      return 0;
    }
  }

  public Flux<BallColor> getBallColor() {
    return sink.asFlux().repeat();
  }

  private int findNextComma(byte[] data, int readLen, int lastComma) {
    do {
      if (readLen <= lastComma + 1) {
        return readLen;
      }
      lastComma++;
    } while (data[lastComma] != ',');
    return lastComma;
  }

  private void threadMain() {
    // Using JNI for a non allocating read
    int port = SerialPortJNI.serialInitializePort((byte) 1);
    SerialPortJNI.serialSetBaudRate(port, 115200);
    SerialPortJNI.serialSetDataBits(port, (byte) 8);
    SerialPortJNI.serialSetParity(port, (byte) 0);
    SerialPortJNI.serialSetStopBits(port, (byte) 10);

    SerialPortJNI.serialSetTimeout(port, 1);
    SerialPortJNI.serialEnableTermination(port, '\n');

    HAL.report(FRCNetComm.tResourceType.kResourceType_SerialPort, 2);

    byte[] buffer = new byte[257];
    SingleCharSequence charSeq = new SingleCharSequence();
    charSeq.data = buffer;
    IntRef lastComma = new IntRef();

    RawColor color0 = new RawColor();

    while (threadRunning.get()) {
      int read = SerialPortJNI.serialRead(port, buffer, buffer.length - 1);
      if (read <= 0) {
        threadLock.lock();
        try {
          this.hasColor0 = false;
        } finally {
          threadLock.unlock();
        }
        continue;
      }
      if (!threadRunning.get()) {
        break;
      }

      // Trim trailing newline if exists
      if (buffer[read - 1] == '\n') {
        read--;
      }

      if (read == 0) {
        continue;
      }

      if (debugPrints.get()) {
        System.out.println(new String(buffer, 0, read, StandardCharsets.UTF_8));
      }

      lastComma.value = -1;

      boolean hasColor0 = parseIntFromIndex(charSeq, read, lastComma) != 0;
      color0.red = parseIntFromIndex(charSeq, read, lastComma);
      color0.green = parseIntFromIndex(charSeq, read, lastComma);
      color0.blue = parseIntFromIndex(charSeq, read, lastComma);
      color0.ir = parseIntFromIndex(charSeq, read, lastComma);
      int prox0 = parseIntFromIndex(charSeq, read, lastComma);

      double ts = Timer.getFPGATimestamp();

      threadLock.lock();
      try {
        this.lastReadTime = ts;
        this.hasColor0 = hasColor0;
        if (hasColor0) {
          this.color0.red = color0.red;
          this.color0.green = color0.green;
          this.color0.blue = color0.blue;
          this.color0.ir = color0.ir;

          this.prox0 = prox0;
          sink.tryEmitNext(getPresentBallColor());
        }
      } finally {
        threadLock.unlock();
      }
    }

    SerialPortJNI.serialClose(port);
  }

  public BallColor getPresentBallColor() {
    RawColor color = getRawColor0();
    if (getProximity0() > 10 && (float) color.blue / color.red > 1.5) {
      return BallColor.BLUE;
    } else if (getProximity0() > 10 && (float) color.red / color.blue > 1.5) {
      return BallColor.RED;
    }
    return BallColor.UNKNOWN;
  }

  public boolean isSensor0Connected() {
    try {
      threadLock.lock();
      return hasColor0;
    } finally {
      threadLock.unlock();
    }
  }

  public RawColor getRawColor0() {
    try {
      threadLock.lock();
      return new RawColor(color0.red, color0.green, color0.blue, color0.ir);
    } finally {
      threadLock.unlock();
    }
  }

  public void getRawColor0(RawColor rawColor) {
    try {
      threadLock.lock();
      rawColor.red = color0.red;
      rawColor.green = color0.green;
      rawColor.blue = color0.blue;
      rawColor.ir = color0.ir;
    } finally {
      threadLock.unlock();
    }
  }

  public int getProximity0() {
    try {
      threadLock.lock();
      return prox0;
    } finally {
      threadLock.unlock();
    }
  }

  public double getLastReadTimestampSeconds() {
    try {
      threadLock.lock();
      return lastReadTime;
    } finally {
      threadLock.unlock();
    }
  }

  void setDebugPrints(boolean debug) {
    debugPrints.set(debug);
  }

  @Override
  public void close() throws Exception {
    threadRunning.set(false);
    readThread.join();
  }

  public enum BallColor {
    RED,
    BLUE,
    UNKNOWN
  }

  public static class RawColor {
    public int red;
    public int green;
    public int blue;
    public int ir;

    public RawColor(int r, int g, int b, int _ir) {
      red = r;
      green = g;
      blue = b;
      ir = _ir;
    }

    public RawColor() {}
  }

  private static class SingleCharSequence implements CharSequence {
    public byte[] data;

    @Override
    public int length() {
      return data.length;
    }

    @Override
    public char charAt(int index) {
      return (char) data[index];
    }

    @Override
    public CharSequence subSequence(int start, int end) {
      return new String(data, start, end, StandardCharsets.UTF_8);
    }
  }

  private static class IntRef {
    int value;
  }
}
