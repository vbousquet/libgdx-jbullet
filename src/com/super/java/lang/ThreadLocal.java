package java.lang;

public class ThreadLocal<T> {
	private T value;
	private boolean set;
	
	protected T initialValue() {
		return null;
	}
	
	public T get() {
		if (!set) {
			value = initialValue();
			set = true;
		}
		return value;
	}
	
	public void remove() {
		set = false;
	}
	
	public void set(T value) {
		this.value = value;
		set = true;
	}
}