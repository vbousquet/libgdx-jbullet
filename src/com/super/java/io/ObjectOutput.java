package java.io;

public interface ObjectOutput {
  void writeInt(int value) throws IOException;
  void writeObject(Object value) throws IOException;
}