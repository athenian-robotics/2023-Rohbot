package com.arc852.lib.controllers;

import com.arc852.Robot;
import edu.wpi.first.wpilibj.Joystick;
import java.util.Map;
import java.util.stream.Stream;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

public class FightStick extends Joystick {
  private final Sinks.Many<Map.Entry<FightStick.Button, Boolean>> buttonsSink;

  public FightStick(int port) {
    super(port);
    Robot.getInstance().addPeriodic(this::run);
    buttonsSink = Sinks.many().multicast().onBackpressureBuffer();
  }

  private void run() {
    Stream.of(Button.values())
        .forEach(button -> buttonsSink.tryEmitNext(Map.entry(button, getButton(button))));
  }

  public Flux<Boolean> streamA() {
    return streamButtons().filter(x -> x.getKey() == Button.A).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamB() {
    return streamButtons().filter(x -> x.getKey() == Button.B).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamX() {
    return streamButtons().filter(x -> x.getKey() == Button.X).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamY() {
    return streamButtons().filter(x -> x.getKey() == Button.Y).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamLB() {
    return streamButtons().filter(x -> x.getKey() == Button.LB).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamRB() {
    return streamButtons().filter(x -> x.getKey() == Button.RB).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamL3() {
    return streamButtons().filter(x -> x.getKey() == Button.L3).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamR3() {
    return streamButtons().filter(x -> x.getKey() == Button.R3).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamShare() {
    return streamButtons().filter(x -> x.getKey() == Button.SHARE).map(Map.Entry::getValue);
  }

  public Flux<Boolean> streamOptions() {
    return streamButtons().filter(x -> x.getKey() == Button.OPTIONS).map(Map.Entry::getValue);
  }

  private Flux<Map.Entry<Button, Boolean>> streamButtons() {
    return buttonsSink.asFlux().repeat();
  }

  public boolean getButton(Button button) {
    return getRawButton(button.value);
  }

  public boolean getAButton() {
    return getRawButton(Button.A.value);
  }

  public boolean getBButton() {
    return getRawButton(Button.B.value);
  }

  public boolean getXButton() {
    return getRawButton(Button.X.value);
  }

  public boolean getYButton() {
    return getRawButton(Button.Y.value);
  }

  public boolean getLBButton() {
    return getRawButton(Button.LB.value);
  }

  public boolean getRBButton() {
    return getRawButton(Button.RB.value);
  }

  public boolean getShareButton() {
    return getRawButton(Button.SHARE.value);
  }

  public boolean getOptionsButton() {
    return getRawButton(Button.OPTIONS.value);
  }

  public boolean getL3Button() {
    return getRawButton(Button.L3.value);
  }

  public boolean getR3Button() {
    return getRawButton(Button.R3.value);
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    SHARE(7),
    OPTIONS(8),
    L3(9),
    R3(10);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }
}
