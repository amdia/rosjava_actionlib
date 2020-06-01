package eu.test.utils;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.logging.LogManager;
import java.util.logging.Logger;

/**
 * Created at 2019-05-09
 * @deprecated Does not seem to apply in rosjava
 * @author Spyros Koukas
 */
@Deprecated
public final class LoggerConfigurator {
    private static final String DEFAULT_LOGGING_CONFIG_FILE = "log-config.properties";

    /**
     *
     */
    private LoggerConfigurator() {
        throw new AssertionError("This is a utility class. It should not be instantiated.");
    }

    /**
     * Sets to the default config file {@link LoggerConfigurator#DEFAULT_LOGGING_CONFIG_FILE}
     */
    public static final void configureLogging() {
        configureLogging(DEFAULT_LOGGING_CONFIG_FILE);
    }

    /**
     * @param pathToLoggingFile
     */
    public static final void configureLogging(final String pathToLoggingFile) {
        final InputStream inputStream = LoggerConfigurator.class.getResourceAsStream(pathToLoggingFile);
        try {
            if(inputStream!=null){
            LogManager.getLogManager().readConfiguration(inputStream);}

            else{
                throw new FileNotFoundException("File: "+pathToLoggingFile+" was not found.");
            }
        } catch (final IOException e) {
            Logger.getAnonymousLogger().severe("Could not load default logging properties file:" + pathToLoggingFile);
            Logger.getAnonymousLogger().severe(e.getMessage());
        }
    }
}
