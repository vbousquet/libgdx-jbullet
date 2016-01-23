package com.bulletphysics.util;

public class Misc {

  public static native int randInt2(int n) /*-{
    return ~~(Math.random() * n);
  }-*/;
}
