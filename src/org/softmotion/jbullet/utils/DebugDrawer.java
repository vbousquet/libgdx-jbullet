
package org.softmotion.jbullet.utils;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.linearmath.IDebugDraw;

public class DebugDrawer extends IDebugDraw {

	private final ShapeRenderer debugShapes;
	private int debugMode;

	public DebugDrawer () {
		debugShapes = new ShapeRenderer();
		debugShapes.setAutoShapeType(true);
	}

	public void begin (Matrix4 projection, Matrix4 transform) {
		Gdx.gl.glEnable(GL20.GL_BLEND);
		debugShapes.setProjectionMatrix(projection);
		debugShapes.setTransformMatrix(transform);
		debugShapes.begin();

	}

	public void end () {
		debugShapes.end();
	}

	@Override
	public void drawLine (Vector3 from, Vector3 to, Vector3 color) {
		debugShapes.setColor(color.x, color.y, color.z, 1);
		debugShapes.line(from.x * 64, from.y * 64, to.x * 64, to.y * 64);
	}

	@Override
	public void drawContactPoint (Vector3 PointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color) {
		// System.err.println("Draw contact : " + PointOnB + " -> " + normalOnB + " [" + color + "]");
	}

	@Override
	public void reportErrorWarning (String warningString) {
	}

	@Override
	public void draw3dText (Vector3 location, String textString) {
	}

	@Override
	public void setDebugMode (int debugMode) {
		this.debugMode = debugMode;
	}

	@Override
	public int getDebugMode () {
		return debugMode;
	}

}
