package com.bulletphysics.util;

import com.google.gwt.core.client.JavaScriptObject;

import java.io.Externalizable;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectOutput;
import java.util.AbstractList;
import java.util.RandomAccess;

public final class ObjectArrayList<T> extends AbstractList<T> implements RandomAccess, Externalizable {

	private JavaScriptObject array;
	private int capacity;
	
	public ObjectArrayList() {
		this(16);
	}
	
	@SuppressWarnings("unchecked")
	public ObjectArrayList(int initialCapacity) {
		array = JavaScriptObject.createArray();
		capacity = initialCapacity;
	}
	
	@Override
	public native boolean add(T value) /*-{
		this.@com.bulletphysics.util.ObjectArrayList::array.push(value);
		return true;
	}-*/;

	@Override
	public native void add(int index, T value) /*-{
		if (this.@com.bulletphysics.util.ObjectArrayList::array.length == index) {
			this.@com.bulletphysics.util.ObjectArrayList::array.push(value);
		} else {
			this.@com.bulletphysics.util.ObjectArrayList::array.splice(index, 0, value);
		}
	}-*/;

	@Override
	public native T remove(int index) /*-{
		if (index == this.@com.bulletphysics.util.ObjectArrayList::array.length - 1) {
			return this.@com.bulletphysics.util.ObjectArrayList::array.pop();
		} 
		return this.@com.bulletphysics.util.ObjectArrayList::array.splice(index, 1)[0];
    }-*/;
	
	
	public native void removeQuick(int index) /*-{
		if (index == this.@com.bulletphysics.util.ObjectArrayList::array.length - 1) {
			return this.@com.bulletphysics.util.ObjectArrayList::array.pop();
		} 
		return this.@com.bulletphysics.util.ObjectArrayList::array.splice(index, 1)[0];
	}-*/;

	public native T get(int index) /*-{
		return this.@com.bulletphysics.util.ObjectArrayList::array[index];
	}-*/;

	public native T getQuick(int index)  /*-{
		return this.@com.bulletphysics.util.ObjectArrayList::array[index];
	}-*/;

	@Override
	public native T set(int index, T value)  /*-{
		var result = this.@com.bulletphysics.util.ObjectArrayList::array[index];
		this.@com.bulletphysics.util.ObjectArrayList::array[index] = value;
		return result
	}-*/;

	public native void setQuick(int index, T value)  /*-{
		return this.@com.bulletphysics.util.ObjectArrayList::array[index] = value;
	}-*/;

	public native int size() /*-{
		return this.@com.bulletphysics.util.ObjectArrayList::array.length;
	}-*/;
	
	public int capacity() {
		while (capacity < size()) {
			capacity *= 2;
		}
		return capacity;
	}
		
	
	@Override
	public native void clear()/*-{
		this.@com.bulletphysics.util.ObjectArrayList::array.length = 0;
	}-*/;

	@Override
	public int indexOf(Object o) {
		int _size = size();
		if (o == null) {
			for (int i=0; i<_size; i++) {
				if (get(i) == null) {
					return i;
				}
			}
		} else {
			for (int i=0; i<_size; i++) {
				if (o.equals(get(i))) {
					return i;
				}
			}
		}
		
		return -1;
	}

	public void writeExternal(ObjectOutput out) throws IOException {
		out.writeInt(size());
		for (int i=0; i<size(); i++) {
			out.writeObject(get(i));
		}
	}

	public void readExternal(ObjectInput in) throws IOException, ClassNotFoundException {
		clear();
		int size = in.readInt();
		for (int i=0; i<size; i++) {
			add((T)in.readObject());
		}
	}
	
}
