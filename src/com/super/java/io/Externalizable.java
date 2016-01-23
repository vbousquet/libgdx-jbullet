package java.io;

public interface Externalizable {
  void readExternal(ObjectInput in) throws IOException, ClassNotFoundException;
  void writeExternal(ObjectOutput out) throws IOException;
}