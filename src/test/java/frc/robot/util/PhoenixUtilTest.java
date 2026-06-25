package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

public class PhoenixUtilTest {
  @Test
  public void testTryUntilOkStopsAtSuccess() {
    AtomicInteger calls = new AtomicInteger();
    Supplier<StatusCode> supplier =
        () -> {
          int attempt = calls.getAndIncrement();
          return attempt < 3 ? StatusCode.GeneralError : StatusCode.OK;
        };

    PhoenixUtil.tryUntilOk(5, supplier);

    assertEquals(4, calls.get(), "Supplier should be called until success");
  }

  @Test
  public void testTryUntilOkStopsAtMaxAttempts() {
    AtomicInteger calls = new AtomicInteger();
    Supplier<StatusCode> supplier =
        () -> {
          calls.getAndIncrement();
          return StatusCode.GeneralError;
        };

    PhoenixUtil.tryUntilOk(3, supplier);

    assertEquals(3, calls.get(), "Supplier should not exceed maxAttempts");
  }
}
