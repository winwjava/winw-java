package winw.ai.slam;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.time.Instant;

/** Durable diagnostics independent of external process output. */
final class CalibrationRuntime {
    private static final org.slf4j.Logger LOGGER=org.slf4j.LoggerFactory.getLogger(CalibrationRuntime.class);
    private CalibrationRuntime() {}
    static void failure(Path log, Throwable error) throws IOException {
        java.io.StringWriter trace=new java.io.StringWriter();
        error.printStackTrace(new java.io.PrintWriter(trace));
        log(log,"FAILED: "+trace);
    }
    static void log(Path log, String message) throws IOException {
        LOGGER.info("{}",message);
        Files.createDirectories(log.toAbsolutePath().getParent());
        Files.write(log, (Instant.now()+" "+message+System.lineSeparator()).getBytes(StandardCharsets.UTF_8),
                StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    }
}
