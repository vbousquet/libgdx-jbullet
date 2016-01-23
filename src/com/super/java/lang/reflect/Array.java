package java.lang.reflect;

public class Array {
  
  static native void log(String s) /*-{
    $wnd.console.log(s);
  }-*/;
  
  public static Object newInstance(Class<?> componentType, int length) {
    if (componentType == boolean.class) {
      return new boolean[length];
    }
    if (componentType == byte.class) {
      return new byte[length];
    }
    if (componentType == char.class) {
      return new char[length];
    }
    if (componentType == short.class) {
      return new short[length];
    }
    if (componentType == int.class) {
      return new int[length];
    }
    if (componentType == long.class) {
      return new long[length];
    }
    if (componentType == float.class) {
      return new float[length];
    }
    if (componentType == double.class) {
      return new double[length];
    }
    return new Object[length];
  }
}