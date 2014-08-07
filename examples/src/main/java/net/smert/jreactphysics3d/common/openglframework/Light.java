package net.smert.jreactphysics3d.common.openglframework;

import java.nio.FloatBuffer;
import net.smert.jreactphysics3d.common.openglframework.maths.Color;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.EXTFramebufferObject;
import org.lwjgl.opengl.GL11;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Light extends Object3D {

    // OpenGL light ID
    private int mLightID;

    // Diffuse color of the light
    private Color mDiffuseColor;

    // Specular color of the light
    private Color mSpecularColor;

    // True if the light is active
    private boolean mIsActive;

    // Shadow map associated with this light
    private Texture2D mShadowMap;

    // Framebuffer object to render the shadow map
    private FrameBufferObject mFBOShadowMap;

    // Shader to render the depth of the scene into a texture
    private Shader mDepthShader;

    // Return the diffuse color
    public Color getDiffuseColor() {
        return mDiffuseColor;
    }

    // Set the diffuse color
    public void setDiffuseColor(Color color) {
        mDiffuseColor = color;
    }

    // Return the specular color
    public Color getSpecularColor() {
        return mSpecularColor;
    }

    // Set the specular color
    public void setSpecularColor(Color color) {
        mSpecularColor = color;
    }

    // Return true if the light is active
    public boolean getIsActive() {
        return mIsActive;
    }

    // Enable the light
    public void enable() {

        mIsActive = true;

        // Enable the light
        GL11.glEnable(GL11.GL_LIGHTING);
        GL11.glEnable(GL11.GL_LIGHT0 + mLightID);
    }

    // Disable the light
    public void disable() {

        mIsActive = false;

        // Disable the light
        GL11.glDisable(GL11.GL_LIGHT0 + mLightID);
    }

    // Destroy the shadow map associated with this light
    public void destroyShadowMap() {
        mShadowMap.destroy();
        mFBOShadowMap.destroy();
        mDepthShader.destroy();
    }

    // Call this method before rendering the scene for the shadow map computation
    public void startRenderingShadowMap() {
        assert (mShadowMap.getID() != 0);
    }

    // Call this method after having rendered the scene for the shadow map computation
    public void stopRenderingShadowMap() {
        assert (mShadowMap.getID() != 0);
    }

    // Constructor
    public Light(int id) {
        mLightID = id;
        mDiffuseColor = Color.white();
        mSpecularColor = Color.white();
        mIsActive = false;
    }

    // Constructor
    public Light(int id, Color diffuseColor, Color specularColor) {
        mLightID = id;
        mDiffuseColor = diffuseColor;
        mSpecularColor = specularColor;
        mIsActive = false;
    }

    // Initialize the light
    public void init() {

        // Enable the light
        enable();

        // Set the diffuse and specular color
        float diffuseColor[] = {mDiffuseColor.r, mDiffuseColor.g, mDiffuseColor.b, mDiffuseColor.a};
        float specularColor[] = {mSpecularColor.r, mSpecularColor.g, mSpecularColor.b, mSpecularColor.a};
        FloatBuffer fb1 = BufferUtils.createFloatBuffer(4);
        FloatBuffer fb2 = BufferUtils.createFloatBuffer(4);
        fb1.put(diffuseColor);
        fb1.flip();
        fb2.put(specularColor);
        fb2.flip();
        GL11.glLight(mLightID, GL11.GL_DIFFUSE, fb1);
        GL11.glLight(mLightID, GL11.GL_SPECULAR, fb2);
    }

    // Create a shadow map associated with this light
    public boolean createShadowMap(int width, int height) {

        // Destroy the current shadow map
        destroyShadowMap();

        // Create the Framebuffer object to render the shadow map
        boolean isFBOCreated = mFBOShadowMap.create(width, height, false);
        if (!isFBOCreated) {
            System.out.println("Error : Cannot create the Shadow Map !");
            destroyShadowMap();
            return false;
        }

        // Bind the Framebuffer object
        mFBOShadowMap.bind(GL11.GL_NONE);

        // Create the shadow map depth texture
        mShadowMap.create(width, height, GL11.GL_DEPTH_COMPONENT, GL11.GL_DEPTH_COMPONENT, GL11.GL_UNSIGNED_BYTE);

        // Attache the shadow map texture to the Framebuffer object
        mFBOShadowMap.attachTexture(EXTFramebufferObject.GL_DEPTH_ATTACHMENT_EXT, mShadowMap.getID());

        // Unbind the Framebuffer object
        mFBOShadowMap.unbind();

        // TODO : Change the path of the shader here so that it does not depend on the build folder
        boolean isShaderCreated = mDepthShader.create("shaders/depth.vert", "shaders/depth.vert");
        if (!isShaderCreated) {
            System.out.println("Error : Cannot create the Shadow Map !");
            destroyShadowMap();
            return false;
        }

        return true;
    }

}
