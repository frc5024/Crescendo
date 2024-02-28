package com.team5024.lib.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SmarterDashboard {

  /**
   * Checks the table and tells if it contains the specified key.
   *
   * @param key the key to search for
   * @return true if the table as a value assigned to the given key
   */
  public static boolean containsKey(String key) {
    return SmartDashboard.containsKey(key);
  }

  /**
   * Maps the specified key to the specified value in this table.
   * The key can not be null.
   *
   * @param key  the key
   * @param data the value
   * @throws IllegalArgumentException If key is null
   */
  @SuppressWarnings("PMD.CompareObjectsWithEquals")
  public static synchronized void putData(String key, Sendable data) {
    SmartDashboard.putData(key, data);
  }

  /**
   * Maps the specified key (where the key is the name of the {@link Sendable}) to
   * the specified value in this table.
   *
   * @param value the value
   * @throws IllegalArgumentException If key is null
   */
  public static void putData(Sendable value) {
    SmartDashboard.putData(value);
  }

  /**
   * Put a boolean in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBoolean(String key, boolean value) {
    return SmartDashboard.putBoolean(key, value);
  }

  /**
   * Returns the boolean the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static boolean getBoolean(String key, boolean defaultValue) {
    if (!containsKey(key))
      SmartDashboard.putBoolean(key, defaultValue);
    return SmartDashboard.getBoolean(key, defaultValue);
  }

  /**
   * Put a number in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumber(String key, double value) {
    return SmartDashboard.putNumber(key, value);
  }

  /**
   * Returns the number the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static double getNumber(String key, double defaultValue) {
    if (!containsKey(key))
      SmartDashboard.putNumber(key, defaultValue);
    return SmartDashboard.getNumber(key, defaultValue);
  }

  /**
   * Put a string in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putString(String key, String value) {
    return SmartDashboard.putString(key, value);
  }

  /**
   * Returns the string the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static String getString(String key, String defaultValue) {
    if (!containsKey(key))
      SmartDashboard.putString(key, defaultValue);
    return SmartDashboard.getString(key, defaultValue);
  }

  /**
   * Put a boolean array in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBooleanArray(String key, boolean[] value) {
    return SmartDashboard.putBooleanArray(key, value);
  }

  /**
   * Put a boolean array in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBooleanArray(String key, Boolean[] value) {
    return SmartDashboard.putBooleanArray(key, value);
  }

  /**
   * Returns the boolean array the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static boolean[] getBooleanArray(String key, boolean[] defaultValue) {
    if (!containsKey(key))
      putBooleanArray(key, defaultValue);
    return SmartDashboard.getBooleanArray(key, defaultValue);
  }

  /**
   * Returns the boolean array the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static Boolean[] getBooleanArray(String key, Boolean[] defaultValue) {
    if (!containsKey(key))
      putBooleanArray(key, defaultValue);
    return SmartDashboard.getBooleanArray(key, defaultValue);
  }

  /**
   * Put a number array in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumberArray(String key, double[] value) {
    return SmartDashboard.putNumberArray(key, value);
  }

  /**
   * Put a number array in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumberArray(String key, Double[] value) {
    return SmartDashboard.putNumberArray(key, value);
  }

  /**
   * Returns the number array the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static double[] getNumberArray(String key, double[] defaultValue) {
    if (!containsKey(key))
      putNumberArray(key, defaultValue);
    return SmartDashboard.getNumberArray(key, defaultValue);
  }

  /**
   * Returns the number array the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static Double[] getNumberArray(String key, Double[] defaultValue) {
    if (!containsKey(key))
      putNumberArray(key, defaultValue);
    return SmartDashboard.getNumberArray(key, defaultValue);
  }

  /**
   * Put a string array in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putStringArray(String key, String[] value) {
    return SmartDashboard.putStringArray(key, value);
  }

  /**
   * Returns the string array the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static String[] getStringArray(String key, String[] defaultValue) {
    if (!containsKey(key))
      putStringArray(key, defaultValue);
    return SmartDashboard.getStringArray(key, defaultValue);
  }

  /**
   * Put a raw value (byte array) in the table.
   *
   * @param key   the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putRaw(String key, byte[] value) {
    return SmartDashboard.putRaw(key, value);
  }

  /**
   * Returns the raw value (byte array) the key maps to.
   * If the key does not exist, it will add it with the default value.
   * If the key is a different type, it will return the default value.
   *
   * @param key          the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key
   */
  public static byte[] getRaw(String key, byte[] defaultValue) {
    if (!containsKey(key))
      putRaw(key, defaultValue);
    return SmartDashboard.getRaw(key, defaultValue);
  }
}
