package java.io;

public interface ObjectInput {
  int readInt() throws IOException;
  Object readObject() throws IOException, ClassNotFoundException;
}