package frc.robot.lib.leds;

import java.util.concurrent.ConcurrentHashMap;

public class LedSectionConfig {
    private final static ConcurrentHashMap<String, Integer> m_sectionStart = new ConcurrentHashMap<>();
    private final static ConcurrentHashMap<String, Integer> m_sectionEnd = new ConcurrentHashMap<>();

    public static void addSection(String title, int start, int end) {
        m_sectionStart.put(title, start);
        m_sectionEnd.put(title, end);
    }

    public static int getSectionStart(String title) {
        return m_sectionStart.get(title);
    }

    public static int getSectionEnd(String title) {
        return m_sectionEnd.get(title);
     
    }
}
