
package org.softmotion.jbullet.utils;

import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.Touchable;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.DebugDrawModes;

public class DebugDrawerActor extends Actor {

	private final DynamicsWorld dynamicsWorld;

	public DebugDrawerActor (DynamicsWorld dynamicsWorld) {
		this.dynamicsWorld = dynamicsWorld;
		dynamicsWorld.setDebugDrawer(new DebugDrawer());
		dynamicsWorld.getDebugDrawer().setDebugMode(DebugDrawModes.DRAW_WIREFRAME | DebugDrawModes.DRAW_AABB);
		dynamicsWorld.getDebugDrawer().setDebugMode(DebugDrawModes.DRAW_WIREFRAME);
		setTouchable(Touchable.disabled);
	}

	@Override
	public void draw (Batch batch, float parentAlpha) {
		batch.end();
		((DebugDrawer)dynamicsWorld.getDebugDrawer()).begin(batch.getProjectionMatrix(), batch.getTransformMatrix());
		dynamicsWorld.debugDrawWorld();
		((DebugDrawer)dynamicsWorld.getDebugDrawer()).end();
		batch.begin();
	}

}
