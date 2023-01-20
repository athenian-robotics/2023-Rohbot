package com.arc852.lib.controllers;

import com.arc852.Robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Stream;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

public class ARCXboxController extends XboxController {
  private final Sinks.Many<Map.Entry<Axis, Double>> axesSink;
  private final Sinks.Many<Map.Entry<Button, Boolean>> buttonsSink;

  public ARCXboxController(int port) {
    super(port);
    axesSink = Sinks.many().multicast().onBackpressureBuffer();
    buttonsSink = Sinks.many().multicast().onBackpressureBuffer();
    Robot.getInstance()
        .addPeriodic(
            () -> {
              Stream.of(Button.values())
                  .forEach(button -> buttonsSink.tryEmitNext(Map.entry(button, getButton(button))));

              Stream.of(Axis.values())
                  .forEach(axis -> axesSink.tryEmitNext(Map.entry(axis, getAxes(axis))));
            });
  }

  private Flux<Map.Entry<Axis, Double>> streamAxes() {
    return axesSink.asFlux().repeat();
  }

  private Flux<Map.Entry<Button, Boolean>> streamButtons() {
    return buttonsSink.asFlux().repeat();
  }

  public Flux<Boolean> streamA() {
    return streamButtons().filter(x -> x.getKey() == Button.kA).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamB() {
    return streamButtons().filter(x -> x.getKey() == Button.kB).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamX() {
    return streamButtons().filter(x -> x.getKey() == Button.kX).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamY() {
    return streamButtons().filter(x -> x.getKey() == Button.kY).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamLeftBumper() {
    return streamButtons().filter(x -> x.getKey() == Button.kLeftBumper).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamRightBumper() {
    return streamButtons().filter(x -> x.getKey() == Button.kRightBumper).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamBack() {
    return streamButtons().filter(x -> x.getKey() == Button.kBack).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamStart() {
    return streamButtons().filter(x -> x.getKey() == Button.kStart).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamLeftStick() {
    return streamButtons().filter(x -> x.getKey() == Button.kLeftStick).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamRightStick() {
    return streamButtons().filter(x -> x.getKey() == Button.kRightStick).map(Map.Entry::getValue);
  }

  public Flux<Double[]> streamLeftJoystick() {
    return streamAxes()
        .filter(x -> x.getKey() == Axis.kLeftX || x.getKey() == Axis.kLeftY)
        .collectList()
        .map(
            list -> {
              Double[] axes = new Double[2];
              list.forEach(
                  entry -> {
                    if (entry.getKey() == Axis.kLeftX) {
                      axes[0] = entry.getValue();
                    } else if (entry.getKey() == Axis.kLeftY) {
                      axes[1] = entry.getValue();
                    }
                  });
              return axes;
            })
        .repeat();
  }

  public Flux<Double[]> streamRightJoystick() {
    return streamAxes()
        .filter(x -> x.getKey() == Axis.kRightX || x.getKey() == Axis.kRightY)
        .collectList()
        .map(
            list -> {
              Double[] axes = new Double[2];
              list.forEach(
                  entry -> {
                    if (entry.getKey() == Axis.kRightX) {
                      axes[0] = entry.getValue();
                    } else if (entry.getKey() == Axis.kRightY) {
                      axes[1] = entry.getValue();
                    }
                  });
              return axes;
            })
        .repeat();
  }

  public Flux<Double> streamLeftTrigger() {
    return streamAxes().filter(x -> x.getKey() == Axis.kLeftTrigger).map(Map.Entry::getValue);
  }

  public Flux<Double> streamRightTrigger() {
    return streamAxes().filter(x -> x.getKey() == Axis.kRightTrigger).map(Map.Entry::getValue);
  }

  public void bindButton(Button button, Command command) {
    streamButtons()
        .subscribe(
            buttonEntry -> {
              if (buttonEntry.getKey().equals(button) && buttonEntry.getValue()) {
                command.schedule();
              }
            });
  }

  public void bindAxes(Axis axis, Consumer<Double> consumer) {
    streamAxes()
        .subscribe(
            axes -> {
              if (axes.getKey() == axis) {
                consumer.accept(axes.getValue());
              }
            });
  }

  private boolean getButton(Button button) {
    switch (button) {
      case kA:
        return getAButton();
      case kB:
        return getBButton();
      case kX:
        return getXButton();
      case kY:
        return getYButton();
      case kBack:
        return getBackButton();
      case kStart:
        return getStartButton();
      case kLeftBumper:
        return getLeftBumper();
      case kRightBumper:
        return getRightBumper();
      case kLeftStick:
        return getLeftStickButton();
      case kRightStick:
        return getRightStickButton();
      default:
        throw new IllegalArgumentException("Unknown button: " + button);
    }
  }

  private double getAxes(Axis axis) {
    switch (axis) {
      case kLeftX:
        return getLeftX();
      case kRightX:
        return getRightX();
      case kLeftY:
        return getLeftY();
      case kRightY:
        return getRightY();
      case kLeftTrigger:
        return getLeftTriggerAxis();
      case kRightTrigger:
        return getRightTriggerAxis();
      default:
        throw new IllegalArgumentException("Unknown axis: " + axis);
    }
  }
}
